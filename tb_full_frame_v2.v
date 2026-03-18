`timescale 1ns/1ps
// tb_full_frame_v2.v
// ─────────────────────────────────────────────────────────────────────────────
// ALL FIXES APPLIED vs original tb_full_frame.v:
//
//  FIX 1 (was causing 73,771 errors):
//    Added while(!dec_dr) and while(!udec_dr) backpressure waits in decoder
//    replay loops. Without these, decoder received bytes ~9x faster than it
//    could process them and silently skipped every 8th byte.
//
//  FIX 2 (silent data loss — 5,120 Y pixels dropped per frame):
//    Y  uses 60 blocks now (was 50). 176/32=5.5 → 6 block-rows needed.
//    UV uses 60 blocks now (was 50).  88/16=5.5 → 6 block-rows needed.
//    Requires re-running extract_frame_hex_v2.py to regenerate hex files.
//
//  FIX 3 (better compression + speed):
//    Uses loco_ctx_wrap_v2 which wraps loco_mb_enc_v2:
//    - Automatically passes inter-block context (top row + left pixel)
//    - No reset between blocks (context would be lost)
//    - S_SCAN2 removed from encoder (saves 1025 clk/Y block)
//
//  FIX 4 (testbench structure):
//    Encode all 60 Y blocks first, then decode all 60 Y packets.
//    Same for U and V. No interleaved reset between blocks.
//
// REQUIRES:
//   - extract_frame_hex_v2.py output:  frame_y_all.hex (60×1024 bytes)
//                                      frame_u_all.hex (60×256  bytes)
//                                      frame_v_all.hex (60×256  bytes)
// ─────────────────────────────────────────────────────────────────────────────
module tb_full_frame_v2;

reg clk=0, rst_n=0;
always #5 clk=~clk;

// ── Y encoder: ctx_wrap handles inter-block context automatically ─────────────
wire        y_enc_ready;
wire [7:0]  y_pkt_data;
wire        y_pkt_valid, y_pkt_last;
wire        y_enc_done;
wire        y_enc_ovf;
wire [10:0] y_enc_bytes;
reg  [7:0]  y_enc_din; reg y_enc_dv=0;

loco_ctx_wrap_v2 #(.MB_W(32),.MB_H(32),.BUDGET(1028),.BLOCKS_WIDE(10)) Y_WRAP (
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(y_enc_din),.s_axis_tvalid(y_enc_dv),.s_axis_tready(y_enc_ready),
    .m_axis_tdata(y_pkt_data),.m_axis_tvalid(y_pkt_valid),
    .m_axis_tready(1'b1),.m_axis_tlast(y_pkt_last),
    .mb_done(y_enc_done),.mb_overflow(y_enc_ovf),.mb_comp_bytes(y_enc_bytes)
);

// ── Y decoder ─────────────────────────────────────────────────────────────────
wire        dec_dr;
wire [7:0]  dec_dout; wire dec_dv_out, dec_done;
reg  [7:0]  dec_din=0; reg dec_dv=0, dec_dlast=0;

loco_mb_dec_v2 #(.MB_W(32),.MB_H(32),.BUDGET(1028)) Y_DEC (
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(dec_din),.s_axis_tvalid(dec_dv),
    .s_axis_tready(dec_dr),.s_axis_tlast(dec_dlast),
    .m_axis_tdata(dec_dout),.m_axis_tvalid(dec_dv_out),
    .m_axis_tready(1'b1),.mb_done(dec_done)
);

// ── U encoder ─────────────────────────────────────────────────────────────────
wire        uenc_ready;
wire [7:0]  u_pkt_data;
wire        u_pkt_valid, u_pkt_last;
wire        uenc_done, uenc_ovf; wire [10:0] uenc_bytes;
reg  [7:0]  uenc_din; reg uenc_dv=0;

loco_ctx_wrap_v2 #(.MB_W(16),.MB_H(16),.BUDGET(260),.BLOCKS_WIDE(10)) U_WRAP (
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(uenc_din),.s_axis_tvalid(uenc_dv),.s_axis_tready(uenc_ready),
    .m_axis_tdata(u_pkt_data),.m_axis_tvalid(u_pkt_valid),
    .m_axis_tready(1'b1),.m_axis_tlast(u_pkt_last),
    .mb_done(uenc_done),.mb_overflow(uenc_ovf),.mb_comp_bytes(uenc_bytes)
);

// ── V encoder ─────────────────────────────────────────────────────────────────
wire        venc_ready;
wire [7:0]  v_pkt_data;
wire        v_pkt_valid, v_pkt_last;
wire        venc_done, venc_ovf; wire [10:0] venc_bytes;
reg  [7:0]  venc_din; reg venc_dv=0;

loco_ctx_wrap_v2 #(.MB_W(16),.MB_H(16),.BUDGET(260),.BLOCKS_WIDE(10)) V_WRAP (
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(venc_din),.s_axis_tvalid(venc_dv),.s_axis_tready(venc_ready),
    .m_axis_tdata(v_pkt_data),.m_axis_tvalid(v_pkt_valid),
    .m_axis_tready(1'b1),.m_axis_tlast(v_pkt_last),
    .mb_done(venc_done),.mb_overflow(venc_ovf),.mb_comp_bytes(venc_bytes)
);

// ── UV decoder (shared, reset between U and V phases) ────────────────────────
wire        uvdec_dr;
wire [7:0]  uvdec_dout; wire uvdec_dv_out, uvdec_done;
reg  [7:0]  uvdec_din=0; reg uvdec_dv=0, uvdec_dlast=0;

loco_mb_dec_v2 #(.MB_W(16),.MB_H(16),.BUDGET(260)) UV_DEC (
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(uvdec_din),.s_axis_tvalid(uvdec_dv),
    .s_axis_tready(uvdec_dr),.s_axis_tlast(uvdec_dlast),
    .m_axis_tdata(uvdec_dout),.m_axis_tvalid(uvdec_dv_out),
    .m_axis_tready(1'b1),.mb_done(uvdec_done)
);

// ── Frame pixel stores ────────────────────────────────────────────────────────
// 60 blocks each: Y=60×1024, UV=60×256
reg [7:0] y_frame [0:60*1024-1];
reg [7:0] u_frame [0:60*256-1];
reg [7:0] v_frame [0:60*256-1];

// ── Packet buffers: all blocks stored simultaneously ─────────────────────────
reg [7:0] y_pkt_all  [0:60*1028-1]; integer y_pkt_ptr=0;
reg [7:0] u_pkt_all  [0:60*260-1];  integer u_pkt_ptr=0;
reg [7:0] v_pkt_all  [0:60*260-1];  integer v_pkt_ptr=0;

// ── Decoder output stores ─────────────────────────────────────────────────────
reg [7:0] y_dec_all  [0:60*1024-1]; integer y_dec_ptr=0;
reg [7:0] u_dec_all  [0:60*256-1];  integer u_dec_ptr=0;
reg [7:0] v_dec_all  [0:60*256-1];  integer v_dec_ptr=0;

// ── Capture encoder outputs ───────────────────────────────────────────────────
always @(posedge clk) begin
    if (y_pkt_valid) begin y_pkt_all[y_pkt_ptr]=y_pkt_data; y_pkt_ptr=y_pkt_ptr+1; end
    if (u_pkt_valid) begin u_pkt_all[u_pkt_ptr]=u_pkt_data; u_pkt_ptr=u_pkt_ptr+1; end
    if (v_pkt_valid) begin v_pkt_all[v_pkt_ptr]=v_pkt_data; v_pkt_ptr=v_pkt_ptr+1; end
end

// ── Capture decoder outputs ───────────────────────────────────────────────────
always @(posedge clk) begin
    if (dec_dv_out)   begin y_dec_all[y_dec_ptr]=dec_dout;    y_dec_ptr=y_dec_ptr+1;   end
    if (uvdec_dv_out) begin
        // UV_DEC is shared; u_dec_ptr used first, then v_dec_ptr
        // controlled by which phase we are in (see initial block logic)
    end
end

integer timeout, i, j, blk;
integer y_errors=0, u_errors=0, v_errors=0;
integer tot_ry=0,tot_cy=0,tot_ey=0,ovy=0,raty=0;
integer tot_ruv=0,tot_cuv=0,tot_euv=0,ovuv=0,ratuv=0;
integer cap_sz, blk_errors;
reg [7:0] tmp_pkt[0:1027];

// ── Helper: per-block stats from already-encoded pkt_all ─────────────────────
// (printed during decode phase when we compare)

integer uv_dec_ptr_shared;
reg uv_phase_u; // 1=decoding U, 0=decoding V

always @(posedge clk) begin
    if (uvdec_dv_out) begin
        if (uv_phase_u) begin u_dec_all[uv_dec_ptr_shared]=uvdec_dout; uv_dec_ptr_shared=uv_dec_ptr_shared+1; end
        else            begin v_dec_all[uv_dec_ptr_shared]=uvdec_dout; uv_dec_ptr_shared=uv_dec_ptr_shared+1; end
    end
end

initial begin
    uv_phase_u=1; uv_dec_ptr_shared=0;

    $readmemh("frame_y_all.hex", y_frame);
    $readmemh("frame_u_all.hex", u_frame);
    $readmemh("frame_v_all.hex", v_frame);

    $display("==========================================================");
    $display("  FULL FRAME TEST v2  320x176  YUV 4:2:0  (60 blocks each)");
    $display("==========================================================");

    // ── RESET ─────────────────────────────────────────────────────────────────
    rst_n=0; repeat(8) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);

    // ══════════════════════════════════════════════════════════════════════════
    // PHASE 1: ENCODE all 60 Y blocks (no reset between blocks — ctx maintained)
    // ══════════════════════════════════════════════════════════════════════════
    $display("\n--- ENCODING Y (60 blocks) ---");
    for (i=0; i<60*1024; i=i+1) begin
        @(posedge clk);
        while (!y_enc_ready) @(posedge clk);
        y_enc_dv=1; y_enc_din=y_frame[i];
    end
    @(posedge clk); y_enc_dv=0;
    // wait for last block to finish
    timeout=0;
    while (!y_enc_done && timeout<2000000) begin @(posedge clk); timeout=timeout+1; end
    repeat(8) @(posedge clk);
    $display("  Y encode done. Packets: %0d bytes", y_pkt_ptr);

    // ══════════════════════════════════════════════════════════════════════════
    // PHASE 2: ENCODE all 60 U blocks
    // ══════════════════════════════════════════════════════════════════════════
    $display("\n--- ENCODING U (60 blocks) ---");
    for (i=0; i<60*256; i=i+1) begin
        @(posedge clk);
        while (!uenc_ready) @(posedge clk);
        uenc_dv=1; uenc_din=u_frame[i];
    end
    @(posedge clk); uenc_dv=0;
    timeout=0;
    while (!uenc_done && timeout<2000000) begin @(posedge clk); timeout=timeout+1; end
    repeat(8) @(posedge clk);
    $display("  U encode done. Packets: %0d bytes", u_pkt_ptr);

    // ══════════════════════════════════════════════════════════════════════════
    // PHASE 3: ENCODE all 60 V blocks
    // ══════════════════════════════════════════════════════════════════════════
    $display("\n--- ENCODING V (60 blocks) ---");
    for (i=0; i<60*256; i=i+1) begin
        @(posedge clk);
        while (!venc_ready) @(posedge clk);
        venc_dv=1; venc_din=v_frame[i];
    end
    @(posedge clk); venc_dv=0;
    timeout=0;
    while (!venc_done && timeout<2000000) begin @(posedge clk); timeout=timeout+1; end
    repeat(8) @(posedge clk);
    $display("  V encode done. Packets: %0d bytes", v_pkt_ptr);

    // ══════════════════════════════════════════════════════════════════════════
    // PHASE 4: DECODE all 60 Y packets (with backpressure)
    // ══════════════════════════════════════════════════════════════════════════
    $display("\n--- Y CHANNEL (per-block) ---");
    // Feed all 60×1028 bytes to Y_DEC continuously
    for (j=0; j<60*1028; j=j+1) begin
        @(posedge clk);
        while (!dec_dr) @(posedge clk);    // ← correct backpressure wait
        dec_dv=1; dec_din=y_pkt_all[j]; dec_dlast=(j==60*1028-1);
    end
    @(posedge clk); dec_dv=0; dec_dlast=0;
    timeout=0;
    while (!dec_done && timeout<5000000) begin @(posedge clk); timeout=timeout+1; end
    repeat(10) @(posedge clk);

    // Per-block report
    for (blk=0; blk<60; blk=blk+1) begin
        blk_errors=0;
        for (i=0; i<1024; i=i+1)
            if (y_dec_all[blk*1024+i] !== y_frame[blk*1024+i]) blk_errors=blk_errors+1;
        cap_sz = {y_pkt_all[blk*1028+1][1:0], y_pkt_all[blk*1028+2]};
        $display("Y  blk%02d (r%0d,c%0d)  %4dB  %3d%%  k=%0d  ovf=%0d  %s",
            blk, blk/10, blk%10, cap_sz, 100-100*cap_sz/1024,
            y_pkt_all[blk*1028][6:5], y_pkt_all[blk*1028][7],
            blk_errors==0 ? "LOSSLESS" : "ERRORS");
        tot_ry=tot_ry+1024; tot_cy=tot_cy+cap_sz;
        tot_ey=tot_ey+blk_errors;
        raty=raty+(100-100*cap_sz/1024);
        if (y_pkt_all[blk*1028][7]) ovy=ovy+1;
    end

    // ══════════════════════════════════════════════════════════════════════════
    // PHASE 5: DECODE all 60 U packets
    // ══════════════════════════════════════════════════════════════════════════
    $display("\n--- U CHANNEL (per-block) ---");
    uv_phase_u=1; uv_dec_ptr_shared=0;
    rst_n=0; repeat(4) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);
    for (j=0; j<60*260; j=j+1) begin
        @(posedge clk);
        while (!uvdec_dr) @(posedge clk);  // ← correct backpressure wait
        uvdec_dv=1; uvdec_din=u_pkt_all[j]; uvdec_dlast=(j==60*260-1);
    end
    @(posedge clk); uvdec_dv=0; uvdec_dlast=0;
    timeout=0;
    while (!uvdec_done && timeout<5000000) begin @(posedge clk); timeout=timeout+1; end
    repeat(10) @(posedge clk);
    for (blk=0; blk<60; blk=blk+1) begin
        blk_errors=0;
        for (i=0; i<256; i=i+1)
            if (u_dec_all[blk*256+i] !== u_frame[blk*256+i]) blk_errors=blk_errors+1;
        cap_sz = {u_pkt_all[blk*260+1][1:0], u_pkt_all[blk*260+2]};
        $display("U  blk%02d (r%0d,c%0d)  %4dB  %3d%%  k=%0d  ovf=%0d  %s",
            blk, blk/10, blk%10, cap_sz, 100-100*cap_sz/256,
            u_pkt_all[blk*260][6:5], u_pkt_all[blk*260][7],
            blk_errors==0 ? "LOSSLESS" : "ERRORS");
        tot_ruv=tot_ruv+256; tot_cuv=tot_cuv+cap_sz;
        tot_euv=tot_euv+blk_errors;
        ratuv=ratuv+(100-100*cap_sz/256);
        if (u_pkt_all[blk*260][7]) ovuv=ovuv+1;
    end

    // ══════════════════════════════════════════════════════════════════════════
    // PHASE 6: DECODE all 60 V packets
    // ══════════════════════════════════════════════════════════════════════════
    $display("\n--- V CHANNEL (per-block) ---");
    uv_phase_u=0; uv_dec_ptr_shared=0;
    rst_n=0; repeat(4) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);
    for (j=0; j<60*260; j=j+1) begin
        @(posedge clk);
        while (!uvdec_dr) @(posedge clk);  // ← correct backpressure wait
        uvdec_dv=1; uvdec_din=v_pkt_all[j]; uvdec_dlast=(j==60*260-1);
    end
    @(posedge clk); uvdec_dv=0; uvdec_dlast=0;
    timeout=0;
    while (!uvdec_done && timeout<5000000) begin @(posedge clk); timeout=timeout+1; end
    repeat(10) @(posedge clk);
    for (blk=0; blk<60; blk=blk+1) begin
        blk_errors=0;
        for (i=0; i<256; i=i+1)
            if (v_dec_all[blk*256+i] !== v_frame[blk*256+i]) blk_errors=blk_errors+1;
        cap_sz = {v_pkt_all[blk*260+1][1:0], v_pkt_all[blk*260+2]};
        $display("V  blk%02d (r%0d,c%0d)  %4dB  %3d%%  k=%0d  ovf=%0d  %s",
            blk, blk/10, blk%10, cap_sz, 100-100*cap_sz/256,
            v_pkt_all[blk*260][6:5], v_pkt_all[blk*260][7],
            blk_errors==0 ? "LOSSLESS" : "ERRORS");
        tot_ruv=tot_ruv+256; tot_cuv=tot_cuv+cap_sz;
        tot_euv=tot_euv+blk_errors;
        ratuv=ratuv+(100-100*cap_sz/256);
        if (v_pkt_all[blk*260][7]) ovuv=ovuv+1;
    end

    // ══════════════════════════════════════════════════════════════════════════
    // SUMMARY
    // ══════════════════════════════════════════════════════════════════════════
    $display("\n==========================================================");
    $display("  SUMMARY");
    $display("==========================================================");
    $display("  Y  : avg %0d%%  raw %0dB -> %0dB  ovf %0d/60   errors %0d",
        raty/60, tot_ry, tot_cy, ovy, tot_ey);
    $display("  UV : avg %0d%%  raw %0dB -> %0dB  ovf %0d/120  errors %0d",
        ratuv/120, tot_ruv, tot_cuv, ovuv, tot_euv);
    $display("  Overall: %0d%%",
        100-100*(tot_cy+tot_cuv)/(tot_ry+tot_ruv));
    $display("  Total errors: %0d", tot_ey+tot_euv);
    $display("==========================================================");
    $finish;
end
endmodule
