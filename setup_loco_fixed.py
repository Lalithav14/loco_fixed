#!/usr/bin/env python3
import os, subprocess
os.makedirs("/mnt/e/loco_fixed", exist_ok=True)
os.chdir("/mnt/e/loco_fixed")

open("loco_mb_enc.v","w").write("""`timescale 1ns/1ps
module loco_mb_enc #(
    parameter MB_W   = 32,
    parameter MB_H   = 32,
    parameter BUDGET = 512
)(
    input  wire        clk, rst_n,
    input  wire [7:0]  s_axis_tdata,
    input  wire        s_axis_tvalid,
    output reg         s_axis_tready,
    output reg  [7:0]  m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast,
    output reg         mb_done,
    output reg         mb_overflow,
    output reg  [9:0]  mb_comp_bytes
);
    localparam MB_PIX = MB_W * MB_H;
    reg [4:0] col, row;
    reg [7:0] line_buf [0:MB_W-1];
    reg [7:0] prev_px;
    wire [7:0] a    = (col==0)         ? 8'd128 : prev_px;
    wire [7:0] b    = (row==0)         ? 8'd128 : line_buf[col];
    wire [7:0] c    = (row==0||col==0) ? 8'd128 : line_buf[col-1];
    wire [7:0] mn_w = (a<b)?a:b;
    wire [7:0] mx_w = (a<b)?b:a;
    wire [7:0] pred = (c>=mx_w)?mn_w:(c<=mn_w)?mx_w:(a+b-c);

    reg [511:0] acc;
    reg [9:0]   acc_fill;
    reg [8:0]   d9, mapped9;
    reg [7:0]   mag, quot;
    reg         rem;
    reg [9:0]   nb, nfill;
    reg [511:0] nacc, top_byte;

    reg [7:0] out_buf [0:BUDGET-1];
    reg [7:0] raw_buf [0:MB_PIX-1];
    reg [9:0] out_cnt;
    reg       overflow;
    reg [9:0] px_cnt;

    localparam S_SCAN=3'd0,S_FLUSH=3'd1,S_HDR=3'd2,
               S_EMIT=3'd3,S_PAD=3'd4,S_DONE=3'd5;
    reg [2:0]  state;
    reg [9:0]  emit_idx, pad_cnt, content_sz;
    reg [3:0]  hdr_phase;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col<=0; row<=0; prev_px<=0; px_cnt<=0;
            acc<=0; acc_fill<=0; out_cnt<=0; overflow<=0;
            state<=S_SCAN; emit_idx<=0; hdr_phase<=0;
            pad_cnt<=0; content_sz<=0;
            s_axis_tready<=1; m_axis_tvalid<=0; m_axis_tlast<=0;
            mb_done<=0; mb_overflow<=0; mb_comp_bytes<=0;
        end else begin
            mb_done<=0; m_axis_tvalid<=0; m_axis_tlast<=0;
            case(state)
            S_SCAN: begin
                s_axis_tready <= 1;
                if (s_axis_tvalid) begin
                    raw_buf[px_cnt] <= s_axis_tdata;
                    d9 = {1'b0,s_axis_tdata} - {1'b0,pred};
                    if (d9[8]) begin
                        mag     = (~d9[7:0])+8'd1;
                        mapped9 = {1'b0,mag,1'b0}-9'd1;
                    end else begin
                        mag     = d9[7:0];
                        mapped9 = {1'b0,mag,1'b0};
                    end
                    quot  = mapped9[8:1];
                    rem   = mapped9[0];
                    nb    = {2'b0,quot}+10'd2;
                    nacc  = (acc<<nb)|{{510{1'b0}},1'b1,rem};
                    nfill = acc_fill+nb;
                    if (nfill>=10'd8) begin
                        top_byte = nacc>>(nfill-10'd8);
                        if (!overflow && out_cnt<BUDGET) begin
                            out_buf[out_cnt] <= top_byte[7:0];
                            out_cnt          <= out_cnt+10'd1;
                            if (out_cnt==BUDGET-1) overflow<=1;
                        end
                        acc      <= nacc & ((512'd1<<(nfill-10'd8))-512'd1);
                        acc_fill <= nfill-10'd8;
                    end else begin
                        acc      <= nacc;
                        acc_fill <= nfill;
                    end
                    line_buf[col] <= s_axis_tdata;
                    prev_px       <= s_axis_tdata;
                    px_cnt        <= px_cnt+10'd1;
                    if (col==MB_W-1) begin
                        col<=0;
                        if (row==MB_H-1) begin
                            row<=0; s_axis_tready<=0; state<=S_FLUSH;
                        end else row<=row+1;
                    end else col<=col+1;
                end
            end

            S_FLUSH: begin
                // flush partial byte — settled acc/acc_fill from previous cycle
                if (acc_fill>0 && !overflow && out_cnt<BUDGET) begin
                    out_buf[out_cnt] <= acc << (4'd8 - acc_fill[3:0]);
                    out_cnt <= out_cnt+10'd1;
                end
                content_sz <= overflow ? MB_PIX[9:0] : out_cnt+
                              ((acc_fill>0&&!overflow&&out_cnt<BUDGET)?10'd1:10'd0);
                hdr_phase<=0; emit_idx<=0; pad_cnt<=0;
                state<=S_HDR;
            end

            S_HDR: begin
                if (m_axis_tready) begin
                    m_axis_tvalid<=1;
                    case(hdr_phase)
                        4'd0: m_axis_tdata<={overflow,7'b0};
                        4'd1: m_axis_tdata<=out_cnt[9:8];
                        4'd2: begin
                            m_axis_tdata<=out_cnt[7:0]; state<=S_EMIT;
                        end
                    endcase
                    hdr_phase<=hdr_phase+4'd1;
                end
            end

            S_EMIT: begin
                if (m_axis_tready) begin
                    if (emit_idx<content_sz) begin
                        m_axis_tvalid<=1;
                        m_axis_tdata<=overflow?raw_buf[emit_idx]:out_buf[emit_idx];
                        emit_idx<=emit_idx+10'd1;
                    end else state<=S_PAD;
                end
            end

            S_PAD: begin
                if (m_axis_tready) begin
                    m_axis_tvalid<=1;
                    m_axis_tdata <=8'h00;
                    if (10'd3+content_sz+pad_cnt >= BUDGET[9:0]-10'd1) begin
                        m_axis_tlast<=1; state<=S_DONE;
                    end else pad_cnt<=pad_cnt+10'd1;
                end
            end

            S_DONE: begin
                mb_done<=1; mb_overflow<=overflow; mb_comp_bytes<=out_cnt;
                state<=S_SCAN; px_cnt<=0; out_cnt<=0;
                acc<=0; acc_fill<=0; overflow<=0;
                emit_idx<=0; hdr_phase<=0; pad_cnt<=0;
                s_axis_tready<=1;
            end
            endcase
        end
    end
endmodule
""")

open("loco_mb_dec.v","w").write("""`timescale 1ns/1ps
module loco_mb_dec #(
    parameter MB_W   = 32,
    parameter MB_H   = 32,
    parameter BUDGET = 512
)(
    input  wire        clk, rst_n,
    input  wire [7:0]  s_axis_tdata,
    input  wire        s_axis_tvalid,
    output reg         s_axis_tready,
    input  wire        s_axis_tlast,
    output reg  [7:0]  m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast,
    output reg         mb_done
);
    localparam MB_PIX = MB_W * MB_H;
    localparam S_HDR=3'd0,S_RAW=3'd1,S_RICE=3'd2,
               S_DRAIN=3'd3,S_DONE=3'd4;
    reg [2:0] state;
    reg [1:0] hdr_cnt;
    reg       is_raw;
    reg [9:0] comp_size, bytes_rd, px_cnt;
    // Bit reader
    reg [7:0] cur_byte;
    reg [2:0] bit_pos;
    reg       have_byte;
    reg [7:0] q_acc;
    reg       rice_phase;
    // MED context
    reg [4:0] col, row;
    reg [7:0] line_buf [0:MB_W-1];
    reg [7:0] prev_px;
    // scratch
    reg [8:0] mp, rc;
    reg [7:0] mag, rn;
    reg       cb;

    wire [7:0] a    = (col==0)         ? 8'd128 : prev_px;
    wire [7:0] b    = (row==0)         ? 8'd128 : line_buf[col];
    wire [7:0] c    = (row==0||col==0) ? 8'd128 : line_buf[col-1];
    wire [7:0] mn_w = (a<b)?a:b;
    wire [7:0] mx_w = (a<b)?b:a;
    wire [7:0] pred = (c>=mx_w)?mn_w:(c<=mn_w)?mx_w:(a+b-c);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=S_HDR; hdr_cnt<=0; is_raw<=0; comp_size<=0;
            bytes_rd<=0; px_cnt<=0; col<=0; row<=0; prev_px<=0;
            cur_byte<=0; bit_pos<=7; have_byte<=0;
            q_acc<=0; rice_phase<=0;
            s_axis_tready<=1; m_axis_tvalid<=0;
            m_axis_tlast<=0; mb_done<=0;
        end else begin
            m_axis_tvalid<=0; mb_done<=0; m_axis_tlast<=0;
            case(state)

            S_HDR: begin
                s_axis_tready<=1;
                if (s_axis_tvalid) begin
                    case(hdr_cnt)
                        2'd0: is_raw<=s_axis_tdata[7];
                        2'd1: comp_size[9:8]<=s_axis_tdata[1:0];
                        2'd2: begin
                            comp_size[7:0]<=s_axis_tdata;
                            bytes_rd<=0; px_cnt<=0;
                            q_acc<=0; rice_phase<=0; have_byte<=0;
                            state<=is_raw?S_RAW:S_RICE;
                        end
                    endcase
                    hdr_cnt<=hdr_cnt+2'd1;
                end
            end

            S_RAW: begin
                s_axis_tready<=m_axis_tready&&(px_cnt<MB_PIX);
                if (s_axis_tvalid&&s_axis_tready) begin
                    m_axis_tdata<=s_axis_tdata; m_axis_tvalid<=1;
                    prev_px<=s_axis_tdata; line_buf[col]<=s_axis_tdata;
                    if(col==MB_W-1)begin col<=0;
                        if(row==MB_H-1)row<=0; else row<=row+1;
                    end else col<=col+1;
                    px_cnt<=px_cnt+10'd1; bytes_rd<=bytes_rd+10'd1;
                    if(px_cnt==MB_PIX-10'd1)begin
                        m_axis_tlast<=1; state<=S_DRAIN;
                    end
                end
            end

            S_RICE: begin
                // KEY FIX: deassert ready same cycle we accept a byte
                if (!have_byte && s_axis_tvalid && bytes_rd<comp_size) begin
                    cur_byte   <= s_axis_tdata;
                    bit_pos    <= 7;
                    have_byte  <= 1;
                    bytes_rd   <= bytes_rd+10'd1;
                    s_axis_tready <= 0;  // immediately deassert
                end else begin
                    s_axis_tready <= (!have_byte) && (bytes_rd<comp_size);
                end

                if (have_byte && m_axis_tready && px_cnt<MB_PIX) begin
                    cb = cur_byte[bit_pos];
                    if (bit_pos==3'd0) begin
                        have_byte     <= 0;
                        // re-enable ready immediately when byte consumed
                        s_axis_tready <= (bytes_rd<comp_size);
                    end else bit_pos<=bit_pos-3'd1;

                    if (rice_phase==0) begin
                        if (cb==0) q_acc<=q_acc+8'd1;
                        else       rice_phase<=1;
                    end else begin
                        mp = {q_acc,cb};
                        mag = (mp+9'd1)>>1;
                        if(mp[0]) rc={1'b0,pred}-{1'b0,mag};
                        else      rc={1'b0,pred}+{1'b0,mag};
                        rn = rc[8]?8'd0:rc[7:0];
                        m_axis_tdata<=rn; m_axis_tvalid<=1;
                        prev_px<=rn; line_buf[col]<=rn;
                        if(col==MB_W-1)begin col<=0;
                            if(row==MB_H-1)row<=0; else row<=row+1;
                        end else col<=col+1;
                        px_cnt<=px_cnt+10'd1;
                        q_acc<=0; rice_phase<=0;
                        if(px_cnt==MB_PIX-10'd1)begin
                            m_axis_tlast<=1; state<=S_DRAIN;
                        end
                    end
                end
            end

            S_DRAIN: begin
                // consume all remaining bytes until tlast
                s_axis_tready<=1;
                if (s_axis_tvalid&&s_axis_tlast) state<=S_DONE;
            end

            S_DONE: begin
                mb_done<=1; state<=S_HDR; hdr_cnt<=0;
                col<=0; row<=0; prev_px<=0;
            end
            endcase
        end
    end
endmodule
""")

open("tb_loco_fixed.v","w").write("""`timescale 1ns/1ps
module tb_loco_fixed;
    reg clk=0,rst_n=0; always #5 clk=~clk;
    reg [7:0] enc_tdata=0; reg enc_tvalid=0;
    wire enc_tready;
    wire [7:0] pkt_tdata; wire pkt_tvalid,pkt_tlast;
    wire mb_done_enc,mb_ovf; wire [9:0] mb_cbytes;

    loco_mb_enc #(.MB_W(32),.MB_H(32),.BUDGET(512)) ENC(
        .clk(clk),.rst_n(rst_n),
        .s_axis_tdata(enc_tdata),.s_axis_tvalid(enc_tvalid),
        .s_axis_tready(enc_tready),
        .m_axis_tdata(pkt_tdata),.m_axis_tvalid(pkt_tvalid),
        .m_axis_tready(pkt_tready),.m_axis_tlast(pkt_tlast),
        .mb_done(mb_done_enc),.mb_overflow(mb_ovf),
        .mb_comp_bytes(mb_cbytes));

    // FIFO between enc and dec to prevent tready conflicts
    reg [7:0] fifo_mem [0:1023];
    reg [9:0] fifo_wr=0, fifo_rd=0;
    wire fifo_empty = (fifo_wr==fifo_rd);
    wire pkt_tready;

    // enc->fifo: always accept from encoder
    assign pkt_tready = 1'b1;
    always @(posedge clk)
        if (pkt_tvalid) begin
            fifo_mem[fifo_wr[9:0]] <= pkt_tdata;
            fifo_wr <= fifo_wr+1;
        end

    // fifo->dec
    wire [7:0] dec_in_data = fifo_mem[fifo_rd[9:0]];
    wire       dec_in_valid = !fifo_empty;
    wire       dec_in_ready;
    // tlast reconstruction: last byte of each 512-byte packet
    wire       dec_in_last = (fifo_rd[8:0]==9'd511);

    always @(posedge clk)
        if (dec_in_valid && dec_in_ready)
            fifo_rd <= fifo_rd+1;

    wire [7:0] dec_pdata; wire dec_pvalid,dec_plast,dec_done;

    loco_mb_dec #(.MB_W(32),.MB_H(32),.BUDGET(512)) DEC(
        .clk(clk),.rst_n(rst_n),
        .s_axis_tdata(dec_in_data),.s_axis_tvalid(dec_in_valid),
        .s_axis_tready(dec_in_ready),.s_axis_tlast(dec_in_last),
        .m_axis_tdata(dec_pdata),.m_axis_tvalid(dec_pvalid),
        .m_axis_tready(1'b1),.m_axis_tlast(dec_plast),
        .mb_done(dec_done));

    reg [7:0] ref_img[0:1023], dec_out[0:1023];
    integer i,pass,fail,dec_cnt,timeout;
    integer saved_cbytes; reg saved_ovf;

    always @(posedge clk)
        if(dec_pvalid) begin dec_out[dec_cnt]=dec_pdata; dec_cnt=dec_cnt+1; end
    always @(posedge clk)
        if(mb_done_enc) begin saved_cbytes=mb_cbytes; saved_ovf=mb_ovf; end

    task run_test;
        input [255:0] name;
        integer j;
        begin
            dec_cnt=0; pass=0; fail=0; saved_cbytes=0; saved_ovf=0;
            fifo_wr=0; fifo_rd=0;
            rst_n=0; repeat(4)@(posedge clk);
            rst_n=1; repeat(2)@(posedge clk);
            for(j=0;j<1024;j=j+1) begin
                @(posedge clk);
                while(!enc_tready)@(posedge clk);
                enc_tvalid=1; enc_tdata=ref_img[j];
            end
            @(posedge clk); enc_tvalid=0;
            timeout=0;
            while(dec_cnt<1024&&timeout<500000)begin
                @(posedge clk);timeout=timeout+1;
            end
            repeat(30)@(posedge clk);
            for(j=0;j<1024;j=j+1)begin
                if(dec_out[j]===ref_img[j]) pass=pass+1;
                else begin fail=fail+1;
                    if(fail<=2)$display("  MISMATCH[%0d]:got=%0d exp=%0d",
                                        j,dec_out[j],ref_img[j]);
                end
            end
            $display("%-20s Rice:%4d B  Ratio:%3d%%  Ovf:%0d  %s",
                name,saved_cbytes,
                saved_cbytes>0?100-100*saved_cbytes/1024:0,
                saved_ovf,pass==1024?"LOSSLESS":"ERRORS!");
        end
    endtask

    initial begin
        $dumpfile("tb_fixed.vcd");$dumpvars(0,tb_loco_fixed);
        $display("==============================================");
        $display("  LOCO FIXED-LENGTH  32x32  BUDGET=512");
        $display("==============================================");

        for(i=0;i<1024;i=i+1) ref_img[i]=128;
        run_test("FLAT_128");

        for(i=0;i<1024;i=i+1)
            ref_img[i]=(128+(i>>5)-16+(i&32'h1F)-16)&8'hFF;
        run_test("SMOOTH");

        for(i=0;i<1024;i=i+1) ref_img[i]=$urandom&8'hFF;
        run_test("RANDOM");

        $display("==============================================");
        $finish;
    end
endmodule
""")

print("Files written. Compiling...")
r = subprocess.run(
    ["iverilog","-g2012","-o","sim_fixed",
     "loco_mb_enc.v","loco_mb_dec.v","tb_loco_fixed.v"],
    capture_output=True, text=True)
if r.returncode!=0:
    print("ERRORS:\n",r.stderr)
else:
    print("OK. Running...")
    r2=subprocess.run(["vvp","sim_fixed"],capture_output=True,text=True,
                      timeout=120)
    print(r2.stdout)
    if r2.stderr: print("STDERR:",r2.stderr[:300])
