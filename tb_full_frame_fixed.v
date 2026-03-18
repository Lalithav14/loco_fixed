`timescale 1ns/1ps
// FIXED tb_full_frame.v
// ─────────────────────────────────────────────────────────────────────────────
// ROOT CAUSE OF ALL 73,771 ERRORS:
//   The decoder replay loop was feeding 1 byte per clock, but the decoder
//   in S_RICE takes ~10 clocks per byte (1 load + 8 bit-process + handshake).
//   Without waiting for s_axis_tready (dec_dr / udec_dr), the testbench
//   blasted bytes 9x faster than the decoder could accept, so the decoder
//   silently skipped every 8 of 9 bytes and decoded garbage.
//
// FIX (two changes per task, identical to the working tb_loco_fixed.v):
//   1. Declare wire dec_dr / udec_dr and connect s_axis_tready.
//   2. Add  while(!dec_dr)  @(posedge clk);  before each decoder byte feed.
// ─────────────────────────────────────────────────────────────────────────────
module tb_full_frame;

reg clk=0, rst_n=0;
always #5 clk=~clk;

// ── Y encoder ─────────────────────────────────────────────────────────────────
reg  [7:0] enc_din;  reg  enc_dv=0;
wire [7:0] enc_dout; wire enc_dv_out, enc_dlast;
wire       enc_done, enc_ovf; wire [10:0] enc_bytes;

loco_mb_enc #(.MB_W(32),.MB_H(32),.BUDGET(1028)) ENC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(enc_din),.s_axis_tvalid(enc_dv),.s_axis_tready(),
    .m_axis_tdata(enc_dout),.m_axis_tvalid(enc_dv_out),
    .m_axis_tready(1'b1),.m_axis_tlast(enc_dlast),
    .mb_done(enc_done),.mb_overflow(enc_ovf),.mb_comp_bytes(enc_bytes)
);

// ── Y decoder — FIX: connect s_axis_tready to wire dec_dr ─────────────────────
reg  [7:0] dec_din;  reg  dec_dv=0, dec_dlast=0;
wire       dec_dr;   // ← ADDED: was .s_axis_tready() (unconnected)
wire [7:0] dec_dout; wire dec_dv_out, dec_done;

loco_mb_dec #(.MB_W(32),.MB_H(32),.BUDGET(1028)) DEC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(dec_din),.s_axis_tvalid(dec_dv),
    .s_axis_tready(dec_dr),          // ← FIXED: was ()
    .s_axis_tlast(dec_dlast),
    .m_axis_tdata(dec_dout),.m_axis_tvalid(dec_dv_out),
    .m_axis_tready(1'b1),.mb_done(dec_done)
);

// ── UV encoder ────────────────────────────────────────────────────────────────
reg  [7:0] uenc_din;  reg  uenc_dv=0;
wire [7:0] uenc_dout; wire uenc_dv_out, uenc_dlast;
wire       uenc_done, uenc_ovf; wire [10:0] uenc_bytes;

loco_mb_enc #(.MB_W(16),.MB_H(16),.BUDGET(260)) UENC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(uenc_din),.s_axis_tvalid(uenc_dv),.s_axis_tready(),
    .m_axis_tdata(uenc_dout),.m_axis_tvalid(uenc_dv_out),
    .m_axis_tready(1'b1),.m_axis_tlast(uenc_dlast),
    .mb_done(uenc_done),.mb_overflow(uenc_ovf),.mb_comp_bytes(uenc_bytes)
);

// ── UV decoder — FIX: connect s_axis_tready to wire udec_dr ──────────────────
reg  [7:0] udec_din;  reg  udec_dv=0, udec_dlast=0;
wire       udec_dr;   // ← ADDED
wire [7:0] udec_dout; wire udec_dv_out, udec_done;

loco_mb_dec #(.MB_W(16),.MB_H(16),.BUDGET(260)) UDEC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(udec_din),.s_axis_tvalid(udec_dv),
    .s_axis_tready(udec_dr),         // ← FIXED: was ()
    .s_axis_tlast(udec_dlast),
    .m_axis_tdata(udec_dout),.m_axis_tvalid(udec_dv_out),
    .m_axis_tready(1'b1),.mb_done(udec_done)
);

// ── Frame stores ──────────────────────────────────────────────────────────────
reg [7:0] y_frame [0:50*1024-1];
reg [7:0] u_frame [0:50*256-1];
reg [7:0] v_frame [0:50*256-1];

// ── Buffers ───────────────────────────────────────────────────────────────────
reg [7:0] ref_img  [0:1023];
reg [7:0] pkt_buf  [0:1027]; integer pkt_len;
reg [7:0] dec_out  [0:1023]; integer dec_cnt;

reg [7:0] uref_img [0:255];
reg [7:0] upkt_buf [0:259];  integer upkt_len;
reg [7:0] udec_out [0:255];  integer udec_cnt;

always @(posedge clk) begin
    if (enc_dv_out)  begin pkt_buf[pkt_len]   = enc_dout;  pkt_len  = pkt_len+1;  end
    if (dec_dv_out)  begin dec_out[dec_cnt]    = dec_dout;  dec_cnt  = dec_cnt+1;  end
    if (uenc_dv_out) begin upkt_buf[upkt_len]  = uenc_dout; upkt_len = upkt_len+1; end
    if (udec_dv_out) begin udec_out[udec_cnt]  = udec_dout; udec_cnt = udec_cnt+1; end
end

integer timeout, i, j, errors, cap_sz;
integer tot_ry=0,tot_cy=0,tot_ey=0,ovy=0,raty=0;
integer tot_ruv=0,tot_cuv=0,tot_euv=0,ovuv=0,ratuv=0;

// ── Y task ────────────────────────────────────────────────────────────────────
task run_y;
    input integer blk;
    begin
        for(i=0;i<1024;i=i+1) ref_img[i]=y_frame[blk*1024+i];

        rst_n=0; pkt_len=0; dec_cnt=0;
        repeat(4) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);

        // ENCODE (encoder s_axis_tready always 1 in S_SCAN1 — no wait needed)
        for(i=0;i<1024;i=i+1) begin
            @(posedge clk); enc_dv=1; enc_din=ref_img[i];
        end
        @(posedge clk); enc_dv=0;
        timeout=0;
        while(!enc_done&&timeout<2000000)
            begin @(posedge clk); timeout=timeout+1; end
        repeat(4) @(posedge clk);
        cap_sz=enc_bytes;

        // DECODE — FIX: wait for dec_dr before each byte (was missing!)
        for(j=0;j<pkt_len;j=j+1) begin
            @(posedge clk);
            while(!dec_dr) @(posedge clk);   // ← THE FIX: honour backpressure
            dec_dv=1; dec_din=pkt_buf[j]; dec_dlast=(j==pkt_len-1);
        end
        @(posedge clk); dec_dv=0; dec_dlast=0;
        timeout=0;
        while(!dec_done&&timeout<2000000)
            begin @(posedge clk); timeout=timeout+1; end
        @(posedge clk);

        errors=0;
        for(i=0;i<1024;i=i+1)
            if(dec_out[i]!==ref_img[i]) errors=errors+1;

        $display("Y  blk%02d (r%0d,c%0d)  %4dB  %3d%%  k=%0d  ovf=%0d  %s",
            blk,blk/10,blk%10,cap_sz,100-100*cap_sz/1024,
            pkt_buf[0][6:5],enc_ovf,errors==0?"LOSSLESS":"ERRORS");

        tot_ry=tot_ry+1024; tot_cy=tot_cy+cap_sz;
        tot_ey=tot_ey+errors; raty=raty+(100-100*cap_sz/1024);
        if(enc_ovf) ovy=ovy+1;
    end
endtask

// ── UV task ───────────────────────────────────────────────────────────────────
task run_uv;
    input integer blk;
    input integer is_v;
    begin
        for(i=0;i<256;i=i+1)
            uref_img[i]=(is_v==0)?u_frame[blk*256+i]:v_frame[blk*256+i];

        rst_n=0; upkt_len=0; udec_cnt=0;
        repeat(4) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);

        for(i=0;i<256;i=i+1) begin
            @(posedge clk); uenc_dv=1; uenc_din=uref_img[i];
        end
        @(posedge clk); uenc_dv=0;
        timeout=0;
        while(!uenc_done&&timeout<2000000)
            begin @(posedge clk); timeout=timeout+1; end
        repeat(4) @(posedge clk);
        cap_sz=uenc_bytes;

        // DECODE UV — FIX: wait for udec_dr before each byte (was missing!)
        for(j=0;j<upkt_len;j=j+1) begin
            @(posedge clk);
            while(!udec_dr) @(posedge clk);  // ← THE FIX: honour backpressure
            udec_dv=1; udec_din=upkt_buf[j]; udec_dlast=(j==upkt_len-1);
        end
        @(posedge clk); udec_dv=0; udec_dlast=0;
        timeout=0;
        while(!udec_done&&timeout<2000000)
            begin @(posedge clk); timeout=timeout+1; end
        @(posedge clk);

        errors=0;
        for(i=0;i<256;i=i+1)
            if(udec_out[i]!==uref_img[i]) errors=errors+1;

        $display("%s  blk%02d (r%0d,c%0d)  %4dB  %3d%%  k=%0d  ovf=%0d  %s",
            is_v==0?"U":"V",
            blk,blk/10,blk%10,cap_sz,100-100*cap_sz/256,
            upkt_buf[0][6:5],uenc_ovf,errors==0?"LOSSLESS":"ERRORS");

        tot_ruv=tot_ruv+256; tot_cuv=tot_cuv+cap_sz;
        tot_euv=tot_euv+errors; ratuv=ratuv+(100-100*cap_sz/256);
        if(uenc_ovf) ovuv=ovuv+1;
    end
endtask

// ── Main ──────────────────────────────────────────────────────────────────────
integer b;
initial begin
    $readmemh("frame_y_all.hex",y_frame);
    $readmemh("frame_u_all.hex",u_frame);
    $readmemh("frame_v_all.hex",v_frame);

    $display("==========================================================");
    $display("  FULL FRAME TEST  320x176  YUV 4:2:0");
    $display("==========================================================");

    $display("\n--- Y CHANNEL ---");
    for(b=0;b<50;b=b+1) run_y(b);

    $display("\n--- U CHANNEL ---");
    for(b=0;b<50;b=b+1) run_uv(b,0);

    $display("\n--- V CHANNEL ---");
    for(b=0;b<50;b=b+1) run_uv(b,1);

    $display("\n==========================================================");
    $display("  SUMMARY");
    $display("==========================================================");
    $display("  Y  : avg %0d%%  raw %0dB -> %0dB  ovf %0d/50   errors %0d",
        raty/50,tot_ry,tot_cy,ovy,tot_ey);
    $display("  UV : avg %0d%%  raw %0dB -> %0dB  ovf %0d/100  errors %0d",
        ratuv/100,tot_ruv,tot_cuv,ovuv,tot_euv);
    $display("  Overall: %0d%%",
        100-100*(tot_cy+tot_cuv)/(tot_ry+tot_ruv));
    $display("  Total errors: %0d", tot_ey+tot_euv);
    $display("==========================================================");
    $finish;
end
endmodule
