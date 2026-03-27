`timescale 1ns/1ps
// loco_ctx_wrap_v2.v
// Wraps loco_mb_enc_v2 and automatically manages inter-block context.
// Feed blocks in raster order (left-to-right, top-to-bottom).
// On each mb_done:
//   - saves out_bottom_flat → used as top_ctx for next block-row
//   - saves out_right_px   → used as left_ctx for next block in same row
//   - advances blk_col, resets at end of row
// No reset between blocks — encoder automatically restarts in S_DONE→S_SCAN1.
//
// Parameters:
//   MB_W, MB_H, BUDGET  — same as loco_mb_enc_v2
//   BLOCKS_WIDE         — blocks per frame row (320/32=10 for Y, 160/16=10 for UV)
module loco_ctx_wrap_v2 #(
    parameter MB_W        = 32,
    parameter MB_H        = 32,
    parameter BUDGET      = 1028,
    parameter BLOCKS_WIDE = 10
)(
    input  wire        clk, rst_n,
    input  wire [7:0]  s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    output wire [7:0]  m_axis_tdata,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready,
    output wire        m_axis_tlast,
    output wire        mb_done,
    output wire        mb_overflow,
    output wire [10:0] mb_comp_bytes
);
    // ── top-row context store: one MB_W*8-bit entry per block-column ─────────
    reg [MB_W*8-1:0] top_row_store [0:BLOCKS_WIDE-1];

    // ── block column counter ──────────────────────────────────────────────────
    reg [$clog2(BLOCKS_WIDE):0] blk_col;

    // ── context valid flags ───────────────────────────────────────────────────
    reg top_ctx_valid_r, left_ctx_valid_r;
    reg [7:0] left_ctx_px_r;

    // ── encoder outputs ───────────────────────────────────────────────────────
    wire [MB_W*8-1:0] out_bottom_flat;
    wire [7:0]        out_right_px;

    // ── encoder instance ──────────────────────────────────────────────────────
    loco_mb_enc_v2 #(.MB_W(MB_W),.MB_H(MB_H),.BUDGET(BUDGET)) ENC (
        .clk(clk),.rst_n(rst_n),
        .s_axis_tdata(s_axis_tdata),.s_axis_tvalid(s_axis_tvalid),
        .s_axis_tready(s_axis_tready),
        .top_ctx_valid(top_ctx_valid_r),
        .top_ctx_flat(top_row_store[blk_col]),
        .left_ctx_valid(left_ctx_valid_r),
        .left_ctx_px(left_ctx_px_r),
        .m_axis_tdata(m_axis_tdata),.m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready),.m_axis_tlast(m_axis_tlast),
        .mb_done(mb_done),.mb_overflow(mb_overflow),.mb_comp_bytes(mb_comp_bytes),
        .out_bottom_flat(out_bottom_flat),.out_right_px(out_right_px)
    );

    // ── context update on each mb_done ────────────────────────────────────────
    integer cx;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            blk_col          <= 0;
            top_ctx_valid_r  <= 0;  // no top context for first block-row
            left_ctx_valid_r <= 0;  // no left context for first block of each row
            left_ctx_px_r    <= 8'd128;
        end else if (mb_done) begin
            // save bottom row → becomes top context for block below
            top_row_store[blk_col] <= out_bottom_flat;
            // save right pixel → becomes left context for next block
            left_ctx_px_r <= out_right_px;

            if (blk_col == BLOCKS_WIDE-1) begin
                // end of block-row: reset to start of next row
                blk_col          <= 0;
                left_ctx_valid_r <= 0;   // first block of row has no left
                top_ctx_valid_r  <= 1;   // all subsequent rows have top context
            end else begin
                blk_col          <= blk_col + 1;
                left_ctx_valid_r <= 1;   // next block has left context
                // top_ctx_valid_r unchanged
            end
        end
    end
endmodule
