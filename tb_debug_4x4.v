`timescale 1ns/1ps
module tb_debug_4x4;
    reg clk=0,rst_n=0; always #5 clk=~clk;

    reg [7:0] enc_in=0; reg enc_v=0;
    wire enc_r, enc_bv, enc_last, mb_done_enc, mb_ovf;
    wire [7:0] enc_byte; wire [9:0] mb_cbytes;

    loco_mb_enc #(.MB_W(4),.MB_H(4),.BUDGET(32)) ENC(
        .clk(clk),.rst_n(rst_n),
        .s_axis_tdata(enc_in),.s_axis_tvalid(enc_v),
        .s_axis_tready(enc_r),
        .m_axis_tdata(enc_byte),.m_axis_tvalid(enc_bv),
        .m_axis_tready(1'b1),.m_axis_tlast(enc_last),
        .mb_done(mb_done_enc),.mb_overflow(mb_ovf),
        .mb_comp_bytes(mb_cbytes));

    reg [7:0] dec_din=0; reg dec_dv=0; reg dec_dlast=0;
    wire dec_dr, dec_pv, dec_done;
    wire [7:0] dec_pix;

    loco_mb_dec #(.MB_W(4),.MB_H(4),.BUDGET(32)) DEC(
        .clk(clk),.rst_n(rst_n),
        .s_axis_tdata(dec_din),.s_axis_tvalid(dec_dv),
        .s_axis_tready(dec_dr),.s_axis_tlast(dec_dlast),
        .m_axis_tdata(dec_pix),.m_axis_tvalid(dec_pv),
        .m_axis_tready(1'b1),.m_axis_tlast(),.mb_done(dec_done));

    reg [7:0] pkt[0:63];
    reg [7:0] dout[0:15];
    integer pkt_len, dec_cnt, i, t;

    // Capture encoder output
    always @(posedge clk)
        if (enc_bv) begin
            pkt[pkt_len] = enc_byte;
            pkt_len      = pkt_len+1;
        end

    // Capture decoder output
    always @(posedge clk)
        if (dec_pv) begin
            dout[dec_cnt] = dec_pix;
            dec_cnt       = dec_cnt+1;
        end

    initial begin
        pkt_len=0; dec_cnt=0;
        rst_n=0; repeat(4)@(posedge clk);
        rst_n=1; repeat(2)@(posedge clk);

        // Feed 16 pixels of 128
        for(i=0;i<16;i=i+1) begin
            @(posedge clk);
            while(!enc_r) @(posedge clk);
            enc_v=1; enc_in=8'd128;
        end
        @(posedge clk); enc_v=0;

        // Wait for encoder done
        t=0; while(!mb_done_enc&&t<5000) begin @(posedge clk);t=t+1;end
        repeat(5)@(posedge clk);
        $display("Rice=%0d Ovf=%0d pkt_len=%0d",mb_cbytes,mb_ovf,pkt_len);
        for(i=0;i<pkt_len;i=i+1) $display("  pkt[%0d]=0x%02h",i,pkt[i]);

        // Feed packet to decoder
        for(i=0;i<pkt_len;i=i+1) begin
            @(posedge clk);
            while(!dec_dr) @(posedge clk);
            dec_dv=1; dec_din=pkt[i];
            dec_dlast=(i==pkt_len-1);
        end
        @(posedge clk); dec_dv=0; dec_dlast=0;

        t=0; while(!dec_done&&t<5000) begin @(posedge clk);t=t+1;end
        repeat(5)@(posedge clk);

        $display("Decoded %0d pixels:",dec_cnt);
        for(i=0;i<16;i=i+1)
            $display("  [%0d] got=%0d exp=128",i,dout[i]);
        $finish;
    end
endmodule
