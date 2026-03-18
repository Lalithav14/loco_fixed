`timescale 1ns/1ps
module tb_diag;

reg clk=0, rst_n=0;
always #5 clk=~clk;

reg  [7:0] enc_din; reg enc_dv=0;
wire [7:0] enc_dout; wire enc_dv_out, enc_dlast;
wire       enc_done, enc_ovf; wire [10:0] enc_bytes;
wire [7:0] dec_dout; wire dec_dv_out, dec_done;

loco_mb_enc #(.MB_W(32),.MB_H(32),.BUDGET(1028)) ENC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(enc_din),.s_axis_tvalid(enc_dv),.s_axis_tready(),
    .m_axis_tdata(enc_dout),.m_axis_tvalid(enc_dv_out),
    .m_axis_tready(1'b1),.m_axis_tlast(enc_dlast),
    .mb_done(enc_done),.mb_overflow(enc_ovf),.mb_comp_bytes(enc_bytes)
);
loco_mb_dec #(.MB_W(32),.MB_H(32),.BUDGET(1028)) DEC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(enc_dout),.s_axis_tvalid(enc_dv_out),
    .s_axis_tready(),.s_axis_tlast(enc_dlast),
    .m_axis_tdata(dec_dout),.m_axis_tvalid(dec_dv_out),
    .m_axis_tready(1'b1),.mb_done(dec_done)
);

reg [7:0] y_frame [0:50*1024-1];
reg [7:0] ref_img [0:1023];
reg [7:0] dec_out [0:1023];
reg [7:0] pkt_buf [0:1027];
integer   dec_cnt, pkt_len;

always @(posedge clk) begin
    if (enc_dv_out) begin pkt_buf[pkt_len] = enc_dout; pkt_len = pkt_len+1; end
    if (dec_dv_out) begin dec_out[dec_cnt] = dec_dout; dec_cnt = dec_cnt+1; end
end

integer timeout, i, errors;

task run_block;
    input integer blk;
    integer sz, kv, ovf_flag;
    begin
        $display("\n--- BLOCK %0d ---", blk);

        // load ref
        for(i=0;i<1024;i=i+1) ref_img[i] = y_frame[blk*1024+i];
        $display("  ref[0..7]: %0d %0d %0d %0d %0d %0d %0d %0d",
            ref_img[0],ref_img[1],ref_img[2],ref_img[3],
            ref_img[4],ref_img[5],ref_img[6],ref_img[7]);

        dec_cnt=0; pkt_len=0;
        @(posedge clk); @(posedge clk);

        for(i=0;i<1024;i=i+1) begin
            @(posedge clk); enc_dv=1; enc_din=ref_img[i];
        end
        @(posedge clk); enc_dv=0;

        timeout=0;
        while(!enc_done&&timeout<2000000)
            begin @(posedge clk); timeout=timeout+1; end
        $display("  enc_done at timeout=%0d  bytes=%0d  pkt_len=%0d",
            timeout, enc_bytes, pkt_len);

        timeout=0;
        while(!dec_done&&timeout<2000000)
            begin @(posedge clk); timeout=timeout+1; end
        $display("  dec_done at timeout=%0d  dec_cnt=%0d", timeout, dec_cnt);

        // wait a few more clocks to let stragglers in
        repeat(10) @(posedge clk);
        $display("  dec_cnt after 10 extra clocks = %0d", dec_cnt);

        sz      = enc_bytes;
        kv      = pkt_buf[0][6:5];
        ovf_flag= pkt_buf[0][7];
        $display("  header: ovf=%0d k=%0d size=%0d blk_mean=%0d",
            ovf_flag, kv, {pkt_buf[1][1:0],pkt_buf[2]}, pkt_buf[3]);
        $display("  dec_out[0..7]: %0d %0d %0d %0d %0d %0d %0d %0d",
            dec_out[0],dec_out[1],dec_out[2],dec_out[3],
            dec_out[4],dec_out[5],dec_out[6],dec_out[7]);

        errors=0;
        for(i=0;i<1024;i=i+1)
            if(dec_out[i]!==ref_img[i]) errors=errors+1;

        if(errors==0) begin
            $display("  LOSSLESS");
        end else begin
            $display("  ERRORS: %0d wrong pixels", errors);
            // print first 5 mismatches
            for(i=0;i<1024;i=i+1)
                if(dec_out[i]!==ref_img[i])
                    $display("    px[%0d] dec=%0d ref=%0d", i, dec_out[i], ref_img[i]);
        end
    end
endtask

initial begin
    $readmemh("frame_y_all.hex", y_frame);
    #12; rst_n=1; #10;

    $display("=== DIAGNOSTIC: testing blk0 then blk1 ===");

    // Test block 0 on its own (should behave like tb_loco_fixed)
    run_block(0);

    $display("\n=== NOW blk1 immediately after (no reset) ===");
    run_block(1);

    $display("\n=== NOW blk1 WITH hard reset first ===");
    rst_n=0; repeat(6) @(posedge clk);
    rst_n=1; repeat(4) @(posedge clk);
    run_block(1);

    $finish;
end
endmodule
