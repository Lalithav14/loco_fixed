`timescale 1ns/1ps
// tb_quick_v3.v — quick 3-Y + 2-UV block test
// Both encoder AND decoder receive inter-block context.
// Use this to verify correctness before running the full frame test.
module tb_quick_v3;

reg clk=0, rst_n=0;
always #5 clk=~clk;

// Y encoder
reg  [7:0]  y_enc_din; reg y_enc_dv=0;
wire        y_enc_ready;
wire [7:0]  y_pkt_data; wire y_pkt_valid, y_pkt_last;
wire        y_enc_done; wire [10:0] y_enc_bytes;
wire [255:0] y_enc_bottom; wire [7:0] y_enc_right;

reg  y_etop_valid=0, y_eleft_valid=0;
reg [255:0] y_etop; reg [7:0] y_eleft=128;

loco_mb_enc_y_v3 #(.MB_W(32),.MB_H(32),.BUDGET(1028)) YENC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(y_enc_din),.s_axis_tvalid(y_enc_dv),.s_axis_tready(y_enc_ready),
    .top_ctx_valid(y_etop_valid),.top_ctx_flat(y_etop),
    .left_ctx_valid(y_eleft_valid),.left_ctx_px(y_eleft),
    .m_axis_tdata(y_pkt_data),.m_axis_tvalid(y_pkt_valid),
    .m_axis_tready(1'b1),.m_axis_tlast(y_pkt_last),
    .mb_done(y_enc_done),.mb_overflow(),.mb_comp_bytes(y_enc_bytes),
    .out_bottom_flat(y_enc_bottom),.out_right_px(y_enc_right)
);

// Y decoder
reg  [7:0]  y_dec_din=0; reg y_dec_dv=0, y_dec_dlast=0;
wire        y_dec_dr;
wire [7:0]  y_dec_dout; wire y_dec_dv_out, y_dec_done;
wire [255:0] y_dec_bottom; wire [7:0] y_dec_right;

reg  y_dtop_valid=0, y_dleft_valid=0;
reg [255:0] y_dtop; reg [7:0] y_dleft=128;

loco_mb_dec_v3 #(.MB_W(32),.MB_H(32),.BUDGET(1028)) YDEC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(y_dec_din),.s_axis_tvalid(y_dec_dv),
    .s_axis_tready(y_dec_dr),.s_axis_tlast(y_dec_dlast),
    .m_axis_tdata(y_dec_dout),.m_axis_tvalid(y_dec_dv_out),
    .m_axis_tready(1'b1),.m_axis_tlast(),.mb_done(y_dec_done),
    .top_ctx_valid(y_dtop_valid),.top_ctx_flat(y_dtop),
    .left_ctx_valid(y_dleft_valid),.left_ctx_px(y_dleft),
    .out_bottom_flat(y_dec_bottom),.out_right_px(y_dec_right)
);

// UV encoder
reg  [7:0]  uv_enc_din; reg uv_enc_dv=0;
wire        uv_enc_ready;
wire [7:0]  uv_pkt_data; wire uv_pkt_valid, uv_pkt_last;
wire        uv_enc_done; wire [10:0] uv_enc_bytes;
wire [127:0] uv_enc_bottom; wire [7:0] uv_enc_right;

reg  uv_etop_valid=0, uv_eleft_valid=0;
reg [127:0] uv_etop; reg [7:0] uv_eleft=128;

loco_mb_enc_uv_v3 #(.MB_W(16),.MB_H(16),.BUDGET(260)) UVENC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(uv_enc_din),.s_axis_tvalid(uv_enc_dv),.s_axis_tready(uv_enc_ready),
    .top_ctx_valid(uv_etop_valid),.top_ctx_flat(uv_etop),
    .left_ctx_valid(uv_eleft_valid),.left_ctx_px(uv_eleft),
    .m_axis_tdata(uv_pkt_data),.m_axis_tvalid(uv_pkt_valid),
    .m_axis_tready(1'b1),.m_axis_tlast(uv_pkt_last),
    .mb_done(uv_enc_done),.mb_overflow(),.mb_comp_bytes(uv_enc_bytes),
    .out_bottom_flat(uv_enc_bottom),.out_right_px(uv_enc_right)
);

// UV decoder
reg  [7:0]  uv_dec_din=0; reg uv_dec_dv=0, uv_dec_dlast=0;
wire        uv_dec_dr;
wire [7:0]  uv_dec_dout; wire uv_dec_dv_out, uv_dec_done;
wire [127:0] uv_dec_bottom; wire [7:0] uv_dec_right;

reg  uv_dtop_valid=0, uv_dleft_valid=0;
reg [127:0] uv_dtop; reg [7:0] uv_dleft=128;

loco_mb_dec_v3 #(.MB_W(16),.MB_H(16),.BUDGET(260)) UVDEC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(uv_dec_din),.s_axis_tvalid(uv_dec_dv),
    .s_axis_tready(uv_dec_dr),.s_axis_tlast(uv_dec_dlast),
    .m_axis_tdata(uv_dec_dout),.m_axis_tvalid(uv_dec_dv_out),
    .m_axis_tready(1'b1),.m_axis_tlast(),.mb_done(uv_dec_done),
    .top_ctx_valid(uv_dtop_valid),.top_ctx_flat(uv_dtop),
    .left_ctx_valid(uv_dleft_valid),.left_ctx_px(uv_dleft),
    .out_bottom_flat(uv_dec_bottom),.out_right_px(uv_dec_right)
);

reg [7:0] y_frame[0:3*1024-1];
reg [7:0] u_frame[0:2*256-1];
reg [7:0] y_pkt[0:1027]; integer y_pkt_len=0;
reg [7:0] y_dec[0:1023]; integer y_dec_cnt=0;
reg [7:0] uv_pkt[0:259]; integer uv_pkt_len=0;
reg [7:0] uv_dec[0:255]; integer uv_dec_cnt=0;

always @(posedge clk) begin
    if(y_pkt_valid)  begin y_pkt[y_pkt_len]=y_pkt_data;   y_pkt_len=y_pkt_len+1; end
    if(y_dec_dv_out) begin y_dec[y_dec_cnt]=y_dec_dout;    y_dec_cnt=y_dec_cnt+1; end
    if(uv_pkt_valid) begin uv_pkt[uv_pkt_len]=uv_pkt_data; uv_pkt_len=uv_pkt_len+1; end
    if(uv_dec_dv_out)begin uv_dec[uv_dec_cnt]=uv_dec_dout; uv_dec_cnt=uv_dec_cnt+1; end
end

integer timeout,i,j,errors;

task run_y;
    input integer blk;
    begin
        $display("--- Y block %0d ---", blk);
        y_pkt_len=0; y_dec_cnt=0;

        // ENCODE
        for(i=0;i<1024;i=i+1) begin
            @(posedge clk); while(!y_enc_ready)@(posedge clk);
            y_enc_dv=1; y_enc_din=y_frame[blk*1024+i];
        end
        @(posedge clk); y_enc_dv=0;
        timeout=0;
        while(!y_enc_done&&timeout<100000) begin @(posedge clk);timeout=timeout+1; end
        // pass encoder context to NEXT encode call
        y_etop<=y_enc_bottom; y_etop_valid<=1;
        y_eleft<=y_enc_right; y_eleft_valid<=0; // single col test
        @(posedge clk);
        $display("  enc: %0dB k=%0d ratio=%0d%%",
            y_enc_bytes,y_pkt[0][6:5],100-100*y_enc_bytes/1024);

        // DECODE — with same context the encoder used
        for(j=0;j<y_pkt_len;j=j+1) begin
            @(posedge clk); while(!y_dec_dr)@(posedge clk);
            y_dec_dv=1; y_dec_din=y_pkt[j]; y_dec_dlast=(j==y_pkt_len-1);
        end
        @(posedge clk); y_dec_dv=0; y_dec_dlast=0;
        timeout=0;
        while(!y_dec_done&&timeout<100000) begin @(posedge clk);timeout=timeout+1; end
        repeat(4)@(posedge clk);
        // pass DECODER context to next decode call
        // (decoded == original since lossless, so contexts match)
        y_dtop<=y_dec_bottom; y_dtop_valid<=1;
        y_dleft<=y_dec_right; y_dleft_valid<=0;

        errors=0;
        for(i=0;i<1024;i=i+1)
            if(y_dec[i]!==y_frame[blk*1024+i]) errors=errors+1;
        if(errors==0) $display("  LOSSLESS");
        else begin
            $display("  ERRORS: %0d wrong pixels",errors);
            for(i=0;i<16;i=i+1)
                if(y_dec[i]!==y_frame[blk*1024+i])
                    $display("    px[%0d] dec=%0d ref=%0d",i,y_dec[i],y_frame[blk*1024+i]);
        end
    end
endtask

task run_uv;
    input integer blk;
    begin
        $display("--- UV block %0d ---", blk);
        uv_pkt_len=0; uv_dec_cnt=0;

        for(i=0;i<256;i=i+1) begin
            @(posedge clk); while(!uv_enc_ready)@(posedge clk);
            uv_enc_dv=1; uv_enc_din=u_frame[blk*256+i];
        end
        @(posedge clk); uv_enc_dv=0;
        timeout=0;
        while(!uv_enc_done&&timeout<50000) begin @(posedge clk);timeout=timeout+1; end
        uv_etop<=uv_enc_bottom; uv_etop_valid<=1;
        @(posedge clk);
        $display("  enc: %0dB k=%0d ratio=%0d%%",
            uv_enc_bytes,uv_pkt[0][6:5],100-100*uv_enc_bytes/256);

        for(j=0;j<uv_pkt_len;j=j+1) begin
            @(posedge clk); while(!uv_dec_dr)@(posedge clk);
            uv_dec_dv=1; uv_dec_din=uv_pkt[j]; uv_dec_dlast=(j==uv_pkt_len-1);
        end
        @(posedge clk); uv_dec_dv=0; uv_dec_dlast=0;
        timeout=0;
        while(!uv_dec_done&&timeout<50000) begin @(posedge clk);timeout=timeout+1; end
        repeat(4)@(posedge clk);
        uv_dtop<=uv_dec_bottom; uv_dtop_valid<=1;

        errors=0;
        for(i=0;i<256;i=i+1)
            if(uv_dec[i]!==u_frame[blk*256+i]) errors=errors+1;
        if(errors==0) $display("  LOSSLESS");
        else $display("  ERRORS: %0d wrong pixels",errors);
    end
endtask

initial begin
    // only load 3 blocks worth — avoids readmemh warning
    $readmemh("frame_y_all.hex",y_frame,0,3*1024-1);
    $readmemh("frame_u_all.hex",u_frame,0,2*256-1);
    $display("=== QUICK TEST v3 ===");
    rst_n=0; repeat(8)@(posedge clk); rst_n=1; repeat(4)@(posedge clk);

    run_y(0); run_y(1); run_y(2);

    // reset between Y and UV test
    rst_n=0; repeat(4)@(posedge clk); rst_n=1; repeat(4)@(posedge clk);
    uv_etop_valid=0; uv_dtop_valid=0;
    run_uv(0); run_uv(1);

    $display("=== DONE — all LOSSLESS = design is correct ===");
    $finish;
end
endmodule
