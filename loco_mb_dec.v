`timescale 1ns/1ps
module loco_mb_dec #(
    parameter MB_W   = 32,
    parameter MB_H   = 32,
    parameter BUDGET = 1028
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
    localparam MB_PIX   = MB_W * MB_H;
    localparam POST_HDR = BUDGET - 4;
    localparam S_HDR=3'd0, S_RAW=3'd1, S_RICE=3'd2,
               S_DRAIN=3'd3, S_DONE=3'd4;

    reg [2:0] state;
    reg [1:0] hdr_cnt;
    reg [7:0] block_mean;
    reg       is_raw;
    reg [1:0] k_val;
    reg [10:0] comp_size, bytes_rd, px_cnt, drain_left;  // 11 bits for BUDGET up to 2047
    reg [7:0] cur_byte;
    reg [2:0] bit_pos;
    reg       have_byte;
    reg [7:0] q_acc;
    reg [1:0] rem_cnt;
    reg [2:0] rem_acc;
    reg       rice_phase;
    reg [4:0] col, row;
    reg [7:0] line_buf [0:MB_W-1];
    reg [7:0] prev_px;
    // Module-level scratch
    reg [8:0] mp, full_rem;
    reg [8:0] rc;
    reg [7:0] mag, rn;
    reg       cb;

    wire [7:0] a    = (col==0)         ? block_mean : prev_px;
    wire [7:0] b    = (row==0)         ? block_mean : line_buf[col];
    wire [7:0] c    = (row==0||col==0) ? block_mean : line_buf[col-1];
    wire [7:0] mn_w = (a<b)?a:b;
    wire [7:0] mx_w = (a<b)?b:a;
    wire [7:0] pred = (c>=mx_w)?mn_w:(c<=mn_w)?mx_w:(a+b-c);

    // Advance pixel position and output
    task emit_px;
        input [7:0] px;
        begin
            m_axis_tdata  <= px;
            m_axis_tvalid <= 1;
            prev_px       <= px;
            line_buf[col] <= px;
            if (col==MB_W-1) begin col<=0;
                if (row==MB_H-1) row<=0; else row<=row+1;
            end else col<=col+1;
            px_cnt<=px_cnt+10'd1;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=S_HDR; block_mean<=8'd128; hdr_cnt<=0; is_raw<=0; k_val<=1;
            comp_size<=0; bytes_rd<=0; px_cnt<=0; drain_left<=0;
            col<=0; row<=0; prev_px<=0;
            cur_byte<=0; bit_pos<=7; have_byte<=0;
            q_acc<=0; rem_cnt<=0; rem_acc<=0; rice_phase<=0;
            s_axis_tready<=1; m_axis_tvalid<=0;
            m_axis_tlast<=0; mb_done<=0;
        end else begin
            m_axis_tvalid<=0; mb_done<=0; m_axis_tlast<=0;
            case(state)

            S_HDR: begin
                s_axis_tready<=1;
                if (s_axis_tvalid) begin
                    case(hdr_cnt)
                        2'd0: begin is_raw<=s_axis_tdata[7]; k_val<=s_axis_tdata[6:5]; end
                        2'd1: comp_size[9:8]<=s_axis_tdata[1:0];
                        2'd2: comp_size[7:0]<=s_axis_tdata;
                        2'd3: begin
                            block_mean<=s_axis_tdata;
                            bytes_rd<=0; px_cnt<=0;
                            q_acc<=0; rem_cnt<=0; rem_acc<=0;
                            rice_phase<=0; have_byte<=0;
                            if (is_raw) begin
                                drain_left<=(POST_HDR>MB_PIX)?(POST_HDR-MB_PIX):0;
                                state<=S_RAW;
                            end else state<=S_RICE;
                        end
                    endcase
                    hdr_cnt<=hdr_cnt+2'd1;
                end
            end

            S_RAW: begin
                s_axis_tready<=m_axis_tready&&(px_cnt<MB_PIX);
                if (s_axis_tvalid&&s_axis_tready) begin
                    emit_px(s_axis_tdata);
                    bytes_rd<=bytes_rd+10'd1;
                    if (px_cnt==MB_PIX-10'd1) begin
                        m_axis_tlast<=1;
                        state<=drain_left>0?S_DRAIN:S_DONE;
                    end
                end
            end

            S_RICE: begin
                s_axis_tready<=(!have_byte)&&(bytes_rd<comp_size);
                if (!have_byte&&s_axis_tvalid&&bytes_rd<comp_size) begin
                    cur_byte<=s_axis_tdata; bit_pos<=7;
                    have_byte<=1; bytes_rd<=bytes_rd+10'd1;
                end
                if (have_byte&&m_axis_tready&&px_cnt<MB_PIX) begin
                    cb=cur_byte[bit_pos];
                    if (bit_pos==3'd0) have_byte<=0;
                    else bit_pos<=bit_pos-3'd1;

                    if (rice_phase==0) begin
                        if (cb==0) q_acc<=q_acc+8'd1;
                        else begin
                            if (k_val==0) begin
                                // k=0: no remainder bits
                                mp = {1'b0,q_acc};
                                mag=(mp+9'd1)>>1;
                                if(mp[0]) rc={1'b0,pred}-{1'b0,mag};
                                else      rc={1'b0,pred}+{1'b0,mag};
                                rn=rc[8]?8'd0:rc[7:0];
                                emit_px(rn);
                                q_acc<=0;
                                if(px_cnt==MB_PIX-10'd1) begin
                                    m_axis_tlast<=1;
                                    drain_left<=POST_HDR-comp_size;
                                    state<=S_DRAIN;
                                end
                            end else begin
                                rice_phase<=1; rem_cnt<=0; rem_acc<=0;
                            end
                        end
                    end else begin
                        // Collect k_val remainder bits MSB first
                        rem_acc<={rem_acc[1:0],cb};
                        if (rem_cnt==(k_val-2'd1)) begin
                            // All remainder bits collected
                            case(k_val)
                                2'd1: full_rem={8'b0,cb};
                                2'd2: full_rem={7'b0,rem_acc[0],cb};
                                2'd3: full_rem={6'b0,rem_acc[1:0],cb};
                                default: full_rem=9'd0;
                            endcase
                            mp=({1'b0,q_acc}<<k_val)|full_rem;
                            mag=(mp+9'd1)>>1;
                            if(mp[0]) rc={1'b0,pred}-{1'b0,mag};
                            else      rc={1'b0,pred}+{1'b0,mag};
                            rn=rc[8]?8'd0:rc[7:0];
                            emit_px(rn);
                            q_acc<=0; rem_cnt<=0; rem_acc<=0; rice_phase<=0;
                            if(px_cnt==MB_PIX-10'd1) begin
                                m_axis_tlast<=1;
                                drain_left<=POST_HDR-comp_size;
                                state<=S_DRAIN;
                            end
                        end else rem_cnt<=rem_cnt+2'd1;
                    end
                end
            end

            S_DRAIN: begin
                s_axis_tready<=(drain_left>0);
                if (drain_left==0) state<=S_DONE;
                else if (s_axis_tvalid) begin
                    drain_left<=drain_left-10'd1;
                    if (drain_left==10'd1) state<=S_DONE;
                end
            end

            S_DONE: begin
                mb_done<=1; state<=S_HDR; hdr_cnt<=0;
                col<=0; row<=0; prev_px<=0;
            end
            endcase
        end
    end
endmodule
