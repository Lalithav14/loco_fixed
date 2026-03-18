`timescale 1ns/1ps
module tb_verbose;

reg clk=0, rst_n=0;
always #5 clk=~clk;

reg  [7:0]  enc_din; reg enc_dv=0;
wire [7:0]  enc_dout; wire enc_dout_v, enc_dlast;
wire        mb_done_enc, mb_ovf;
wire [10:0] mb_cbytes;
wire [7:0]  dec_dout; wire dec_dout_v;
wire        mb_done_dec;

loco_mb_enc #(.MB_W(32),.MB_H(32),.BUDGET(1028)) ENC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(enc_din),.s_axis_tvalid(enc_dv),.s_axis_tready(),
    .m_axis_tdata(enc_dout),.m_axis_tvalid(enc_dout_v),
    .m_axis_tready(1'b1),.m_axis_tlast(enc_dlast),
    .mb_done(mb_done_enc),.mb_overflow(mb_ovf),.mb_comp_bytes(mb_cbytes)
);
loco_mb_dec #(.MB_W(32),.MB_H(32),.BUDGET(1028)) DEC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(enc_dout),.s_axis_tvalid(enc_dout_v),
    .s_axis_tready(),.s_axis_tlast(enc_dlast),
    .m_axis_tdata(dec_dout),.m_axis_tvalid(dec_dout_v),
    .m_axis_tready(1'b1),.mb_done(mb_done_dec)
);

reg [7:0] ref_img [0:1023];
reg [7:0] pkt_buf [0:1027];
reg [7:0] dec_out [0:1023];

// gated counters — only capture when enabled
reg        cap_enc_en=0, cap_dec_en=0;
integer    pkt_len=0, dec_cnt=0;
integer    timeout, i, fail;
integer    px_min, px_max, px_sum;
integer    sz, k, bm, ovf, ratio;

always @(posedge clk)
    if (cap_enc_en && enc_dout_v)
        begin pkt_buf[pkt_len]=enc_dout; pkt_len=pkt_len+1; end

always @(posedge clk)
    if (cap_dec_en && dec_dout_v)
        begin dec_out[dec_cnt]=dec_dout; dec_cnt=dec_cnt+1; end

task run_block;
    input [255:0] name;
    begin
        // hard reset between blocks
        rst_n=0; cap_enc_en=0; cap_dec_en=0;
        pkt_len=0; dec_cnt=0;
        repeat(4) @(posedge clk);
        rst_n=1;
        repeat(4) @(posedge clk);

        // input stats
        px_min=255; px_max=0; px_sum=0;
        for (i=0;i<1024;i=i+1) begin
            if (ref_img[i]<px_min) px_min=ref_img[i];
            if (ref_img[i]>px_max) px_max=ref_img[i];
            px_sum=px_sum+ref_img[i];
        end

        // enable capture, feed encoder
        cap_enc_en=1; cap_dec_en=1;
        for (i=0;i<1024;i=i+1) begin
            @(posedge clk); enc_dv=1; enc_din=ref_img[i];
        end
        @(posedge clk); enc_dv=0;

        // wait for encoder done
        timeout=0;
        while (!mb_done_enc && timeout<2000000)
            begin @(posedge clk); timeout=timeout+1; end

        // wait for decoder done
        timeout=0;
        while (!mb_done_dec && timeout<2000000)
            begin @(posedge clk); timeout=timeout+1; end

        @(posedge clk);
        cap_enc_en=0; cap_dec_en=0;

        // parse header
        ovf   = pkt_buf[0][7];
        k     = pkt_buf[0][6:5];
        sz    = {pkt_buf[1][1:0], pkt_buf[2]};
        bm    = pkt_buf[3];
        ratio = ovf ? 0 : (100 - 100*sz/1024);

        // lossless check
        fail=0;
        for (i=0;i<1024;i=i+1)
            if (dec_out[i]!==ref_img[i]) fail=fail+1;

        // print report
        $display("\n+-----------------------------------------------------+");
        $display("| BLOCK: %-44s|", name);
        $display("+----------------------+------------------------------+");
        $display("| INPUT                | OUTPUT (ENCODER)             |");
        $display("+----------------------+------------------------------+");
        $display("| Min pixel   : %3d    | Rice bytes  : %4d           |", px_min, sz);
        $display("| Max pixel   : %3d    | Ratio       : %3d%%          |", px_max, ratio);
        $display("| Mean pixel  : %3d    | k selected  : %1d             |", px_sum/1024, k);
        $display("| Range       : %3d    | block_mean  : %3d           |", px_max-px_min, bm);
        $display("| Pixels      : 1024   | Overflow    : %1d             |", ovf);
        $display("| Size        : 32x32  | Pkt total   : %4d B         |", pkt_len);
        $display("+----------------------+------------------------------+");
        $display("| FIRST 8 PAYLOAD BYTES (after 4-byte header):        |");
        $display("|   %02x %02x %02x %02x %02x %02x %02x %02x                         |",
            pkt_buf[4],pkt_buf[5],pkt_buf[6],pkt_buf[7],
            pkt_buf[8],pkt_buf[9],pkt_buf[10],pkt_buf[11]);
        $display("+-----------------------------------------------------+");
        if (fail==0)
        $display("| DECODER: LOSSLESS  all 1024 pixels match            |");
        else begin
        $display("| DECODER: ERRORS    %4d pixels wrong                 |", fail);
        for (i=0;i<1024&&fail>0;i=i+1)
            if (dec_out[i]!==ref_img[i]) begin
                $display("|   px[%4d] row=%2d col=%2d  enc=%3d dec=%3d         |",
                    i,i/32,i%32,ref_img[i],dec_out[i]);
                fail=fail-1;
                if (fail==0) i=1024;
            end
        end
        $display("+-----------------------------------------------------+");
    end
endtask

initial begin
    $display("\n=== ENCODER / DECODER VERBOSE TRACE ===\n");

    for (i=0;i<1024;i=i+1) ref_img[i]=128;
    run_block("FLAT_128");

    $readmemh("mb_corridor_y.hex", ref_img);
    run_block("CORRIDOR_Y");

    $readmemh("mb_parking_y.hex", ref_img);
    run_block("PARKING_Y");

    $readmemh("mb_night_y.hex", ref_img);
    run_block("NIGHT_Y");

    $display("\n=== DONE ===");
    $finish;
end
endmodule
