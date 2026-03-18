`timescale 1ns/1ps
module tb_loco_fixed;
    reg clk=0,rst_n=0; always #5 clk=~clk;

    reg [7:0] enc_tdata=0; reg enc_tvalid=0;
    wire enc_tready;
    wire [7:0] pkt_tdata; wire pkt_tvalid,pkt_tlast;
    wire mb_done_enc,mb_ovf; wire [10:0] mb_cbytes;

    loco_mb_enc #(.MB_W(32),.MB_H(32),.BUDGET(1028)) ENC(
        .clk(clk),.rst_n(rst_n),
        .s_axis_tdata(enc_tdata),.s_axis_tvalid(enc_tvalid),
        .s_axis_tready(enc_tready),
        .m_axis_tdata(pkt_tdata),.m_axis_tvalid(pkt_tvalid),
        .m_axis_tready(1'b1),.m_axis_tlast(pkt_tlast),
        .mb_done(mb_done_enc),.mb_overflow(mb_ovf),
        .mb_comp_bytes(mb_cbytes));

    reg [7:0] dec_din=0; reg dec_dv=0; reg dec_dlast=0;
    wire dec_dr; wire [7:0] dec_pdata; wire dec_pvalid,dec_plast,dec_done;

    loco_mb_dec #(.MB_W(32),.MB_H(32),.BUDGET(1028)) DEC(
        .clk(clk),.rst_n(rst_n),
        .s_axis_tdata(dec_din),.s_axis_tvalid(dec_dv),
        .s_axis_tready(dec_dr),.s_axis_tlast(dec_dlast),
        .m_axis_tdata(dec_pdata),.m_axis_tvalid(dec_pvalid),
        .m_axis_tready(1'b1),.m_axis_tlast(dec_plast),
        .mb_done(dec_done));

    reg [7:0] u_enc_tdata=0; reg u_enc_tvalid=0;
    wire u_enc_tready;
    wire [7:0] u_pkt_tdata; wire u_pkt_tvalid,u_pkt_tlast;
    wire u_mb_done_enc,u_mb_ovf; wire [10:0] u_mb_cbytes;

    loco_mb_enc #(.MB_W(16),.MB_H(16),.BUDGET(260)) U_ENC(
        .clk(clk),.rst_n(rst_n),
        .s_axis_tdata(u_enc_tdata),.s_axis_tvalid(u_enc_tvalid),
        .s_axis_tready(u_enc_tready),
        .m_axis_tdata(u_pkt_tdata),.m_axis_tvalid(u_pkt_tvalid),
        .m_axis_tready(1'b1),.m_axis_tlast(u_pkt_tlast),
        .mb_done(u_mb_done_enc),.mb_overflow(u_mb_ovf),
        .mb_comp_bytes(u_mb_cbytes));

    reg [7:0] u_dec_din=0; reg u_dec_dv=0; reg u_dec_dlast=0;
    wire u_dec_dr; wire [7:0] u_dec_pdata; wire u_dec_pvalid,u_dec_done;

    loco_mb_dec #(.MB_W(16),.MB_H(16),.BUDGET(260)) U_DEC(
        .clk(clk),.rst_n(rst_n),
        .s_axis_tdata(u_dec_din),.s_axis_tvalid(u_dec_dv),
        .s_axis_tready(u_dec_dr),.s_axis_tlast(u_dec_dlast),
        .m_axis_tdata(u_dec_pdata),.m_axis_tvalid(u_dec_pvalid),
        .m_axis_tready(1'b1),.m_axis_tlast(),.mb_done(u_dec_done));

    reg [7:0] pkt_buf[0:2047]; integer pkt_len;
    reg [7:0] u_pkt_buf[0:511]; integer u_pkt_len;
    reg [7:0] ref_img[0:1023]; reg [7:0] dec_out[0:1023];
    reg [7:0] u_ref[0:255];    reg [7:0] u_dec_out[0:255];
    integer dec_cnt,u_dec_cnt,pass,fail,timeout,i,j;

    always @(posedge clk) begin
        if(pkt_tvalid)   begin pkt_buf[pkt_len]=pkt_tdata;       pkt_len=pkt_len+1;     end
        if(u_pkt_tvalid) begin u_pkt_buf[u_pkt_len]=u_pkt_tdata; u_pkt_len=u_pkt_len+1; end
        if(dec_pvalid)   begin dec_out[dec_cnt]=dec_pdata;        dec_cnt=dec_cnt+1;     end
        if(u_dec_pvalid) begin u_dec_out[u_dec_cnt]=u_dec_pdata;  u_dec_cnt=u_dec_cnt+1; end
    end

    // Separate registers to capture cbytes — written in initial block
    integer cap_cbytes, cap_ovf;
    integer u_cap_cbytes, u_cap_ovf;

    task run_y;
        input [255:0] name;
        begin
            dec_cnt=0; pkt_len=0; cap_cbytes=0; cap_ovf=0;
            rst_n=0; repeat(6)@(posedge clk);
            rst_n=1; repeat(2)@(posedge clk);
            for(j=0;j<1024;j=j+1) begin
                @(posedge clk);
                while(!enc_tready) @(posedge clk);
                enc_tvalid=1; enc_tdata=ref_img[j];
            end
            @(posedge clk); enc_tvalid=0;
            // Wait for done, then read cbytes on NEXT clock
            timeout=0;
            while(!mb_done_enc&&timeout<2000000) begin @(posedge clk);timeout=timeout+1; end
            @(posedge clk);  // one extra — cbytes NBA settled
            cap_cbytes = mb_cbytes;
            cap_ovf    = mb_ovf;
            repeat(10)@(posedge clk);
            // Decode
            for(j=0;j<pkt_len;j=j+1) begin
                @(posedge clk);
                while(!dec_dr) @(posedge clk);
                dec_dv=1; dec_din=pkt_buf[j]; dec_dlast=(j==pkt_len-1);
            end
            @(posedge clk); dec_dv=0; dec_dlast=0;
            timeout=0;
            while(!dec_done&&timeout<2000000) begin @(posedge clk);timeout=timeout+1; end
            repeat(10)@(posedge clk);
            pass=0; fail=0;
            for(j=0;j<1024;j=j+1) begin
                if(dec_out[j]===ref_img[j]) pass=pass+1;
                else begin fail=fail+1;
                    if(fail<=2) $display("  MM[%0d] got=%0d exp=%0d",j,dec_out[j],ref_img[j]);
                end
            end
            $display("Y %-20s Rice:%4dB Ratio:%3d%% Ovf:%0d %s",
                name, cap_cbytes,
                cap_cbytes>0 ? 100-100*cap_cbytes/1024 : 0,
                cap_ovf, pass==1024?"LOSSLESS":"ERRORS!");
        end
    endtask

    task run_uv;
        input [255:0] name;
        begin
            u_dec_cnt=0; u_pkt_len=0; u_cap_cbytes=0; u_cap_ovf=0;
            rst_n=0; repeat(6)@(posedge clk);
            rst_n=1; repeat(2)@(posedge clk);
            for(j=0;j<256;j=j+1) begin
                @(posedge clk);
                while(!u_enc_tready) @(posedge clk);
                u_enc_tvalid=1; u_enc_tdata=u_ref[j];
            end
            @(posedge clk); u_enc_tvalid=0;
            timeout=0;
            while(!u_mb_done_enc&&timeout<2000000) begin @(posedge clk);timeout=timeout+1; end
            @(posedge clk);
            u_cap_cbytes = u_mb_cbytes;
            u_cap_ovf    = u_mb_ovf;
            repeat(10)@(posedge clk);
            for(j=0;j<u_pkt_len;j=j+1) begin
                @(posedge clk);
                while(!u_dec_dr) @(posedge clk);
                u_dec_dv=1; u_dec_din=u_pkt_buf[j]; u_dec_dlast=(j==u_pkt_len-1);
            end
            @(posedge clk); u_dec_dv=0; u_dec_dlast=0;
            timeout=0;
            while(!u_dec_done&&timeout<2000000) begin @(posedge clk);timeout=timeout+1; end
            repeat(10)@(posedge clk);
            pass=0; fail=0;
            for(j=0;j<256;j=j+1) begin
                if(u_dec_out[j]===u_ref[j]) pass=pass+1;
                else begin fail=fail+1;
                    if(fail<=2) $display("  MM[%0d] got=%0d exp=%0d",j,u_dec_out[j],u_ref[j]);
                end
            end
            $display("U %-20s Rice:%4dB Ratio:%3d%% Ovf:%0d %s",
                name, u_cap_cbytes,
                u_cap_cbytes>0 ? 100-100*u_cap_cbytes/256 : 0,
                u_cap_ovf, pass==256?"LOSSLESS":"ERRORS!");
        end
    endtask

    initial begin
        $dumpfile("tb_fixed.vcd"); $dumpvars(0,tb_loco_fixed);
        $display("================================================");
        $display("  LOCO FIXED-LENGTH  YUV 4:2:0");
        $display("================================================");
        $display("--- Y Channel (32x32) ---");

        for(i=0;i<1024;i=i+1) ref_img[i]=128;
        run_y("FLAT_128");

        for(i=0;i<1024;i=i+1)
            ref_img[i]=(128+(i>>5)-16+(i&32'h1F)-16)&8'hFF;
        run_y("SMOOTH");

        for(i=0;i<1024;i=i+1) ref_img[i]=$urandom&8'hFF;
        run_y("RANDOM");

        $readmemh("mb_corridor_y.hex",ref_img); run_y("CORRIDOR_Y");
        $readmemh("mb_parking_y.hex",ref_img);  run_y("PARKING_Y");
        $readmemh("mb_night_y.hex",ref_img);    run_y("NIGHT_Y");

        $display("--- U Channel (16x16) ---");
        $readmemh("mb_corridor_u.hex",u_ref); run_uv("CORRIDOR_U");
        $readmemh("mb_parking_u.hex",u_ref);  run_uv("PARKING_U");
        $readmemh("mb_night_u.hex",u_ref);    run_uv("NIGHT_U");

        $display("--- V Channel (16x16) ---");
        $readmemh("mb_corridor_u.hex",u_ref); run_uv("CORRIDOR_V");
        $readmemh("mb_parking_u.hex",u_ref);  run_uv("PARKING_V");
        $readmemh("mb_night_u.hex",u_ref);    run_uv("NIGHT_V");
        $display("================================================");
        $finish;
    end
endmodule
