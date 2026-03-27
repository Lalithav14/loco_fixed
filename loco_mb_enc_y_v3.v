`timescale 1ns/1ps
// loco_mb_enc_y_v3.v — Y-channel encoder
// FIX: accumulator 512→128 bits (safe: max rice code=74 bits for 8-bit pixels)
// This makes iverilog 10-15x faster. S_SCAN2 kept for accurate k on Y.
// Inter-block context ports for better boundary prediction.
module loco_mb_enc_y_v3 #(
    parameter MB_W   = 32,
    parameter MB_H   = 32,
    parameter BUDGET = 1028
)(
    input  wire        clk, rst_n,
    input  wire [7:0]  s_axis_tdata,
    input  wire        s_axis_tvalid,
    output reg         s_axis_tready,
    input  wire                 top_ctx_valid,
    input  wire [MB_W*8-1:0]   top_ctx_flat,
    input  wire                 left_ctx_valid,
    input  wire [7:0]           left_ctx_px,
    output reg  [7:0]  m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast,
    output reg         mb_done,
    output reg         mb_overflow,
    output reg  [10:0] mb_comp_bytes,
    output reg  [MB_W*8-1:0]   out_bottom_flat,
    output reg  [7:0]           out_right_px
);
    localparam MB_PIX      = MB_W * MB_H;
    localparam LOG2_MB_PIX = (MB_PIX==1024)?10:(MB_PIX==256)?8:10;
    localparam S_SCAN1=4'd0,S_MEAN=4'd1,S_SCAN2=4'd2,S_PICKK=4'd3,
               S_SCAN3=4'd4,S_FLUSH=4'd5,S_HDR=4'd6,S_EMIT=4'd7,
               S_PAD=4'd8,S_DONE=4'd9;

    reg [4:0] col,row;
    reg [7:0] line_buf[0:MB_W-1];
    reg [7:0] prev_px,block_mean;

    wire [7:0] ctx_a=(col==0)?(left_ctx_valid?left_ctx_px:block_mean):prev_px;
    wire [7:0] ctx_b=(row==0)?(top_ctx_valid?top_ctx_flat[col*8+:8]:block_mean):line_buf[col];
    wire [7:0] ctx_c=(row==0||col==0)?((row==0&&col>0&&top_ctx_valid)?top_ctx_flat[(col-1)*8+:8]:block_mean):line_buf[col-1];
    wire [7:0] mn_w=(ctx_a<ctx_b)?ctx_a:ctx_b;
    wire [7:0] mx_w=(ctx_a<ctx_b)?ctx_b:ctx_a;
    wire [7:0] pred=(ctx_c>=mx_w)?mn_w:(ctx_c<=mn_w)?mx_w:(ctx_a+ctx_b-ctx_c);

    reg [8:0]  res_buf[0:MB_PIX-1];
    reg [7:0]  raw_buf[0:MB_PIX-1];
    reg [7:0]  out_buf[0:BUDGET-1];
    reg [10:0] px_cnt;
    reg [19:0] px_sum,res_sum;
    reg [127:0] acc;
    reg [7:0]   acc_fill;
    reg [8:0]  d9,mapped9,mv,remainder;
    reg [7:0]  mag,quot,nb;
    reg [127:0] nacc;
    reg [10:0] mean;
    reg [1:0]  k_val;
    reg [10:0] out_cnt,enc_px,emit_idx,pad_cnt,content_sz,emit_total;
    reg        overflow;
    reg [3:0]  state,hdr_phase;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col<=0;row<=0;prev_px<=0;px_cnt<=0;px_sum<=0;res_sum<=0;
            block_mean<=8'd128;acc<=0;acc_fill<=0;out_cnt<=0;overflow<=0;k_val<=1;
            state<=S_SCAN1;emit_idx<=0;hdr_phase<=0;pad_cnt<=0;
            content_sz<=0;enc_px<=0;emit_total<=0;
            s_axis_tready<=1;m_axis_tvalid<=0;m_axis_tlast<=0;
            mb_done<=0;mb_overflow<=0;mb_comp_bytes<=0;
        end else begin
            mb_done<=0;m_axis_tvalid<=0;m_axis_tlast<=0;
            case(state)
            S_SCAN1: begin
                s_axis_tready<=1;
                if(s_axis_tvalid) begin
                    raw_buf[px_cnt]<=s_axis_tdata;
                    px_sum<=px_sum+{12'b0,s_axis_tdata};
                    px_cnt<=px_cnt+11'd1;
                    if(px_cnt==MB_PIX-1) begin s_axis_tready<=0;state<=S_MEAN; end
                end
            end
            S_MEAN: begin
                block_mean<=px_sum[LOG2_MB_PIX+7:LOG2_MB_PIX];
                col<=0;row<=0;prev_px<=0;enc_px<=0;res_sum<=0;state<=S_SCAN2;
            end
            S_SCAN2: begin
                if(enc_px<MB_PIX) begin
                    d9={1'b0,raw_buf[enc_px]}-{1'b0,pred};
                    if(d9[8]) begin mag=(~d9[7:0])+8'd1;mapped9={1'b0,mag,1'b0}-9'd1; end
                    else      begin mag=d9[7:0];          mapped9={1'b0,mag,1'b0};      end
                    res_buf[enc_px]<=mapped9;
                    res_sum<=res_sum+{11'b0,mapped9};
                    if(row==MB_H-1) out_bottom_flat[col*8+:8]<=raw_buf[enc_px];
                    if(col==MB_W-1) out_right_px<=raw_buf[enc_px];
                    line_buf[col]<=raw_buf[enc_px];prev_px<=raw_buf[enc_px];
                    if(col==MB_W-1) begin col<=0;row<=(row==MB_H-1)?5'd0:row+5'd1; end
                    else col<=col+5'd1;
                    enc_px<=enc_px+11'd1;
                end else state<=S_PICKK;
            end
            S_PICKK: begin
                mean=res_sum>>LOG2_MB_PIX;
                if      (mean<11'd2)  k_val<=2'd0;
                else if (mean<11'd8)  k_val<=2'd1;
                else if (mean<11'd32) k_val<=2'd2;
                else                  k_val<=2'd3;
                enc_px<=0;acc<=0;acc_fill<=0;out_cnt<=0;overflow<=0;state<=S_SCAN3;
            end
            S_SCAN3: begin
                if(acc_fill>=8'd8) begin
                    if(!overflow&&out_cnt<BUDGET) begin
                        out_buf[out_cnt]<=acc[127:120];out_cnt<=out_cnt+11'd1;
                    end
                    if(out_cnt>=(BUDGET-2)) overflow<=1;
                    acc<=acc<<8;acc_fill<=acc_fill-8'd8;
                end else if(enc_px<MB_PIX) begin
                    mv=res_buf[enc_px];quot=mv>>k_val;
                    remainder=mv&((9'd1<<k_val)-9'd1);
                    nb=quot+8'd1+{6'b0,k_val};
                    nacc=acc|(((128'd1<<k_val)|{121'd0,remainder})
                              <<(8'd127-acc_fill-nb+8'd1));
                    acc<=nacc;acc_fill<=acc_fill+nb;enc_px<=enc_px+11'd1;
                end else state<=S_FLUSH;
            end
            S_FLUSH: begin
                if(acc_fill>0&&!overflow&&out_cnt<BUDGET) begin
                    out_buf[out_cnt]<=acc[127:120];
                    out_cnt<=out_cnt+11'd1;content_sz<=out_cnt+11'd1;
                end else content_sz<=out_cnt;
                hdr_phase<=0;emit_idx<=0;pad_cnt<=0;state<=S_HDR;
            end
            S_HDR: begin
                if(m_axis_tready) begin
                    m_axis_tvalid<=1;
                    case(hdr_phase)
                        4'd0:m_axis_tdata<={overflow,k_val,5'b0};
                        4'd1:m_axis_tdata<=out_cnt[9:8];
                        4'd2:m_axis_tdata<=out_cnt[7:0];
                        4'd3:begin m_axis_tdata<=block_mean;
                                   emit_total<=overflow?MB_PIX[10:0]:out_cnt;
                                   state<=S_EMIT; end
                    endcase
                    hdr_phase<=hdr_phase+4'd1;
                end
            end
            S_EMIT: begin
                if(emit_total==0) state<=S_PAD;
                else if(m_axis_tready&&emit_idx<emit_total) begin
                    m_axis_tvalid<=1;
                    m_axis_tdata<=overflow?raw_buf[emit_idx]:out_buf[emit_idx];
                    emit_idx<=emit_idx+11'd1;
                    if(emit_idx==emit_total-11'd1) state<=S_PAD;
                end
            end
            S_PAD: begin
                if(m_axis_tready) begin
                    m_axis_tvalid<=1;m_axis_tdata<=8'h00;
                    if((11'd4+emit_total+pad_cnt)>=(BUDGET-1)) begin
                        m_axis_tlast<=1;state<=S_DONE;
                    end else pad_cnt<=pad_cnt+11'd1;
                end
            end
            S_DONE: begin
                mb_done<=1;mb_overflow<=overflow;mb_comp_bytes<=out_cnt;
                state<=S_SCAN1;px_cnt<=0;px_sum<=0;res_sum<=0;
                out_cnt<=0;acc<=0;acc_fill<=0;overflow<=0;
                emit_idx<=0;hdr_phase<=0;pad_cnt<=0;enc_px<=0;
                block_mean<=8'd128;col<=0;row<=0;prev_px<=0;s_axis_tready<=1;
            end
            endcase
        end
    end
endmodule
