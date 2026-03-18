`timescale 1ns/1ps
// tb_diag_v2.v
// ─────────────────────────────────────────────────────────────────────────────
// FIXES vs original tb_diag.v:
//
//  FIX 1: Encoder output NO LONGER wired directly to decoder.
//    Original wired enc→dec directly, ignoring s_axis_tready, causing the
//    decoder to receive bytes ~9x faster than it could process them.
//    Now uses the proven buffer+replay pattern from tb_loco_fixed.v.
//
//  FIX 2: Uses loco_mb_enc_v2 with context ports tied to 0.
//    Context valid=0 means fallback to block_mean (identical to v1 for
//    isolated block testing). This lets you test the new encoder in isolation
//    before deploying the full ctx_wrap pipeline.
//
//  FIX 3: Proper reset between each run_block call (context cleared).
//    Each block is tested independently. For context testing, use
//    tb_full_frame_v2.v with loco_ctx_wrap_v2 which maintains context.
// ─────────────────────────────────────────────────────────────────────────────
module tb_diag_v2;

reg clk=0, rst_n=0;
always #5 clk=~clk;

// ── Encoder (loco_mb_enc_v2, context tied to 0 for isolated block test) ──────
reg  [7:0] enc_din; reg enc_dv=0;
wire [7:0] enc_dout; wire enc_dv_out, enc_dlast;
wire       enc_ready, enc_done, enc_ovf; wire [10:0] enc_bytes;

loco_mb_enc_v2 #(.MB_W(32),.MB_H(32),.BUDGET(1028)) ENC (
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(enc_din),.s_axis_tvalid(enc_dv),.s_axis_tready(enc_ready),
    .top_ctx_valid(1'b0), .top_ctx_flat(256'b0),
    .left_ctx_valid(1'b0),.left_ctx_px(8'd128),
    .m_axis_tdata(enc_dout),.m_axis_tvalid(enc_dv_out),
    .m_axis_tready(1'b1),.m_axis_tlast(enc_dlast),
    .mb_done(enc_done),.mb_overflow(enc_ovf),.mb_comp_bytes(enc_bytes),
    .out_bottom_flat(),.out_right_px()
);

// ── Decoder (driven from pkt_buf replay, NOT directly from encoder) ───────────
reg  [7:0] dec_din=0; reg dec_dv=0, dec_dlast=0;
wire       dec_dr;
wire [7:0] dec_dout; wire dec_dv_out, dec_done;

loco_mb_dec_v2 #(.MB_W(32),.MB_H(32),.BUDGET(1028)) DEC (
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(dec_din),.s_axis_tvalid(dec_dv),
    .s_axis_tready(dec_dr),.s_axis_tlast(dec_dlast),
    .m_axis_tdata(dec_dout),.m_axis_tvalid(dec_dv_out),
    .m_axis_tready(1'b1),.mb_done(dec_done)
);

reg [7:0] y_frame  [0:60*1024-1];
reg [7:0] ref_img  [0:1023];
reg [7:0] dec_out  [0:1023];
reg [7:0] pkt_buf  [0:1027];
integer   dec_cnt, pkt_len;

always @(posedge clk) begin
    if (enc_dv_out) begin pkt_buf[pkt_len]=enc_dout; pkt_len=pkt_len+1; end
    if (dec_dv_out) begin dec_out[dec_cnt]=dec_dout;  dec_cnt=dec_cnt+1; end
end

integer timeout, i, j, errors;

task run_block;
    input integer blk;
    begin
        $display("\n--- BLOCK %0d ---", blk);
        for (i=0;i<1024;i=i+1) ref_img[i]=y_frame[blk*1024+i];
        $display("  ref[0..7]: %0d %0d %0d %0d %0d %0d %0d %0d",
            ref_img[0],ref_img[1],ref_img[2],ref_img[3],
            ref_img[4],ref_img[5],ref_img[6],ref_img[7]);

        // reset both encoder and decoder, clear buffers
        rst_n=0; pkt_len=0; dec_cnt=0;
        repeat(6) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);

        // ENCODE
        for (i=0;i<1024;i=i+1) begin
            @(posedge clk);
            while (!enc_ready) @(posedge clk);
            enc_dv=1; enc_din=ref_img[i];
        end
        @(posedge clk); enc_dv=0;
        timeout=0;
        while (!enc_done&&timeout<2000000) begin @(posedge clk); timeout=timeout+1; end
        @(posedge clk);
        $display("  enc_done: bytes=%0d pkt_len=%0d k=%0d ovf=%0d mean=%0d",
            enc_bytes, pkt_len,
            pkt_buf[0][6:5], pkt_buf[0][7], pkt_buf[3]);
        repeat(4) @(posedge clk);

        // DECODE — buffer+replay with backpressure (THE CORRECT PATTERN)
        for (j=0;j<pkt_len;j=j+1) begin
            @(posedge clk);
            while (!dec_dr) @(posedge clk);   // wait for decoder ready
            dec_dv=1; dec_din=pkt_buf[j]; dec_dlast=(j==pkt_len-1);
        end
        @(posedge clk); dec_dv=0; dec_dlast=0;
        timeout=0;
        while (!dec_done&&timeout<2000000) begin @(posedge clk); timeout=timeout+1; end
        repeat(10) @(posedge clk);
        $display("  dec_cnt=%0d", dec_cnt);
        $display("  dec_out[0..7]: %0d %0d %0d %0d %0d %0d %0d %0d",
            dec_out[0],dec_out[1],dec_out[2],dec_out[3],
            dec_out[4],dec_out[5],dec_out[6],dec_out[7]);

        errors=0;
        for (i=0;i<1024;i=i+1)
            if (dec_out[i]!==ref_img[i]) errors=errors+1;

        if (errors==0) $display("  LOSSLESS");
        else begin
            $display("  ERRORS: %0d wrong pixels", errors);
            for (i=0;i<1024;i=i+1)
                if (dec_out[i]!==ref_img[i])
                    $display("    px[%0d] dec=%0d ref=%0d", i, dec_out[i], ref_img[i]);
        end
    end
endtask

initial begin
    $readmemh("frame_y_all.hex", y_frame);
    #12; rst_n=1; #10;

    $display("=== DIAGNOSTIC v2: isolated block tests ===");
    run_block(0);
    run_block(1);
    run_block(2);

    $display("\n=== Testing UV block (16x16) ===");
    // For UV testing, instantiate a separate 16x16 encoder/decoder or
    // reuse this testbench with MB_W=16,MB_H=16,BUDGET=260 parameters.

    $finish;
end
endmodule
