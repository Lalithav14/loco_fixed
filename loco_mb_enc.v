`timescale 1ns/1ps
module loco_mb_enc #(
    parameter MB_W   = 32,
    parameter MB_H   = 32,
    parameter BUDGET = 1028
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
    output reg  [10:0] mb_comp_bytes
);
    localparam MB_PIX     = MB_W * MB_H;
    localparam LOG2_MB_PIX = (MB_PIX==1024)?10:(MB_PIX==256)?8:(MB_PIX==64)?6:(MB_PIX==16)?4:10;

    localparam S_SCAN1  = 4'd0;  // pass 1: collect raw pixels + px_sum
    localparam S_MEAN   = 4'd1;  // compute block_mean
    localparam S_SCAN2  = 4'd2;  // pass 2: MED with block_mean → res_buf + res_sum
    localparam S_PICKK  = 4'd3;  // select k from res_sum
    localparam S_SCAN3  = 4'd4;  // pass 3: Rice encode res_buf → out_buf
    localparam S_FLUSH  = 4'd5;  // flush partial byte
    localparam S_HDR    = 4'd6;  // emit 4-byte header
    localparam S_EMIT   = 4'd7;  // emit payload
    localparam S_PAD    = 4'd8;  // emit zero padding
    localparam S_DONE   = 4'd9;

    // MED predictor (used in pass 2 and pass 3)
    reg [4:0] col, row;
    reg [7:0] line_buf [0:MB_W-1];
    reg [7:0] prev_px;
    reg [7:0] block_mean;

    wire [7:0] a    = (col==0)         ? block_mean : prev_px;
    wire [7:0] b    = (row==0)         ? block_mean : line_buf[col];
    wire [7:0] c    = (row==0||col==0) ? block_mean : line_buf[col-1];
    wire [7:0] mn_w = (a<b)?a:b;
    wire [7:0] mx_w = (a<b)?b:a;
    wire [7:0] pred = (c>=mx_w)?mn_w:(c<=mn_w)?mx_w:(a+b-c);

    // Buffers
    reg [8:0]  res_buf [0:MB_PIX-1];
    reg [7:0]  raw_buf [0:MB_PIX-1];
    reg [7:0]  out_buf [0:BUDGET-1];

    // Accumulators
    reg [10:0] px_cnt;
    reg [19:0] px_sum;
    reg [19:0] res_sum;

    // 512-bit MSB-justified accumulator for Rice
    reg [511:0] acc;
    reg [9:0]   acc_fill;

    // Scratch (all module-level, no named blocks)
    reg [8:0]  d9, mapped9, mv, remainder;
    reg [7:0]  mag, quot;
    reg [9:0]  nb;
    reg [511:0] nacc;
    reg [10:0] mean;

    reg [1:0]  k_val;
    reg [10:0] out_cnt, enc_px, emit_idx, pad_cnt, content_sz, emit_total;
    reg        overflow;
    reg [3:0]  state;
    reg [3:0]  hdr_phase;

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col<=0; row<=0; prev_px<=0; px_cnt<=0;
            px_sum<=0; res_sum<=0; block_mean<=8'd128;
            acc<=0; acc_fill<=0; out_cnt<=0; overflow<=0; k_val<=1;
            state<=S_SCAN1; emit_idx<=0; hdr_phase<=0;
            pad_cnt<=0; content_sz<=0; enc_px<=0; emit_total<=0;
            s_axis_tready<=1; m_axis_tvalid<=0; m_axis_tlast<=0;
            mb_done<=0; mb_overflow<=0; mb_comp_bytes<=0;
        end else begin
            mb_done<=0; m_axis_tvalid<=0; m_axis_tlast<=0;
            case(state)

            // PASS 1: accept pixels, store raw, accumulate pixel sum
            S_SCAN1: begin
                s_axis_tready<=1;
                if (s_axis_tvalid) begin
                    raw_buf[px_cnt] <= s_axis_tdata;
                    px_sum          <= px_sum + {12'b0, s_axis_tdata};
                    px_cnt          <= px_cnt + 11'd1;
                    if (px_cnt == MB_PIX-1) begin
                        s_axis_tready <= 0;
                        state         <= S_MEAN;
                    end
                end
            end

            // Compute block_mean, reset MED context for pass 2
            S_MEAN: begin
                block_mean <= px_sum >> LOG2_MB_PIX;
                col<=0; row<=0; prev_px<=0;
                enc_px<=0; res_sum<=0;
                state<=S_SCAN2;
            end

            // PASS 2: re-run MED with block_mean boundary, compute mapped residuals
            S_SCAN2: begin
                if (enc_px < MB_PIX) begin
                    d9 = {1'b0, raw_buf[enc_px]} - {1'b0, pred};
                    if (d9[8]) begin
                        mag     = (~d9[7:0]) + 8'd1;
                        mapped9 = {1'b0, mag, 1'b0} - 9'd1;
                    end else begin
                        mag     = d9[7:0];
                        mapped9 = {1'b0, mag, 1'b0};
                    end
                    res_buf[enc_px] <= mapped9;
                    res_sum         <= res_sum + {11'b0, mapped9};
                    // Update MED context
                    line_buf[col] <= raw_buf[enc_px];
                    prev_px       <= raw_buf[enc_px];
                    if (col == MB_W-1) begin
                        col <= 0;
                        row <= (row == MB_H-1) ? 5'd0 : row + 5'd1;
                    end else col <= col + 5'd1;
                    enc_px <= enc_px + 11'd1;
                end else state <= S_PICKK;
            end

            // Select k from pass-2 residuals (with correct block_mean boundary)
            S_PICKK: begin
                mean = res_sum >> LOG2_MB_PIX;
                if      (mean < 11'd2)  k_val <= 2'd0;
                else if (mean < 11'd8)  k_val <= 2'd1;
                else if (mean < 11'd32) k_val <= 2'd2;
                else                    k_val <= 2'd3;
                enc_px<=0; acc<=0; acc_fill<=0; out_cnt<=0; overflow<=0;
                state<=S_SCAN3;
            end

            // PASS 3: Rice encode res_buf → out_buf
            // Flush MSB byte each clock when full, load pixel only when acc_fill<8
            S_SCAN3: begin
                if (acc_fill >= 10'd8) begin
                    if (!overflow && out_cnt < BUDGET) begin
                        out_buf[out_cnt] <= acc[511:504];
                        out_cnt <= out_cnt + 11'd1;
                    end
                    if (out_cnt >= (BUDGET-2)) overflow <= 1;
                    acc      <= acc << 8;
                    acc_fill <= acc_fill - 10'd8;
                end else if (enc_px < MB_PIX) begin
                    mv        = res_buf[enc_px];
                    quot      = mv >> k_val;
                    remainder = mv & ((9'd1 << k_val) - 9'd1);
                    nb        = {2'b0, quot} + 10'd1 + {8'b0, k_val};
                    nacc      = acc | (((512'd1<<k_val)|{{503{1'b0}},remainder})
                                      << (9'd511 - acc_fill - nb + 10'd1));
                    acc      <= nacc;
                    acc_fill <= acc_fill + nb;
                    enc_px   <= enc_px + 11'd1;
                end else state <= S_FLUSH;
            end

            S_FLUSH: begin
                if (acc_fill > 0 && !overflow && out_cnt < BUDGET) begin
                    out_buf[out_cnt] <= acc[511:504];
                    out_cnt  <= out_cnt + 11'd1;
                    content_sz <= out_cnt + 11'd1;
                end else content_sz <= out_cnt;
                hdr_phase<=0; emit_idx<=0; pad_cnt<=0;
                state<=S_HDR;
            end

            // 4-byte header: flags | size_hi | size_lo | block_mean
            S_HDR: begin
                if (m_axis_tready) begin
                    m_axis_tvalid <= 1;
                    case(hdr_phase)
                        4'd0: m_axis_tdata <= {overflow, k_val, 5'b0};
                        4'd1: m_axis_tdata <= out_cnt[9:8];
                        4'd2: m_axis_tdata <= out_cnt[7:0];
                        4'd3: begin
                            m_axis_tdata <= block_mean;
                            emit_total   <= overflow ? MB_PIX[10:0] : out_cnt;
                            state        <= S_EMIT;
                        end
                    endcase
                    hdr_phase <= hdr_phase + 4'd1;
                end
            end

            S_EMIT: begin
                if (emit_total == 0) state <= S_PAD;
                else if (m_axis_tready && emit_idx < emit_total) begin
                    m_axis_tvalid <= 1;
                    m_axis_tdata  <= overflow ? raw_buf[emit_idx] : out_buf[emit_idx];
                    emit_idx      <= emit_idx + 11'd1;
                    if (emit_idx == emit_total - 11'd1) state <= S_PAD;
                end
            end

            S_PAD: begin
                if (m_axis_tready) begin
                    m_axis_tvalid <= 1; m_axis_tdata <= 8'h00;
                    if ((11'd4 + emit_total + pad_cnt) >= (BUDGET-1)) begin
                        m_axis_tlast <= 1; state <= S_DONE;
                    end else pad_cnt <= pad_cnt + 11'd1;
                end
            end

            S_DONE: begin
                mb_done<=1; mb_overflow<=overflow; mb_comp_bytes<=out_cnt;
                state<=S_SCAN1; px_cnt<=0; px_sum<=0; res_sum<=0;
                out_cnt<=0; acc<=0; acc_fill<=0; overflow<=0;
                emit_idx<=0; hdr_phase<=0; pad_cnt<=0; enc_px<=0;
                block_mean<=8'd128; col<=0; row<=0; prev_px<=0;
                s_axis_tready<=1;
            end
            endcase
        end
    end
endmodule
