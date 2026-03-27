`timescale 1ns/1ps
// tb_full_frame_v3.v
// Uses loco_mb_enc_y_v3  + loco_mb_dec_v3 for Y
// Uses loco_mb_enc_uv_v3 + loco_mb_dec_v3 for UV
// Both encoder AND decoder receive matching inter-block context.
// 60 blocks per channel (extract_frame_hex_v2.py required).
module tb_full_frame_v3;

reg clk=0, rst_n=0;
always #5 clk=~clk;

// ── Y encoder ─────────────────────────────────────────────────────────────────
reg  [7:0]  y_enc_din; reg y_enc_dv=0;
wire        y_enc_ready;
wire [7:0]  y_pkt_data; wire y_pkt_valid;
wire        y_enc_done; wire [10:0] y_enc_bytes; wire y_enc_ovf;
wire [255:0] y_enc_bottom; wire [7:0] y_enc_right;

reg  y_etop_v=0, y_eleft_v=0;
reg [255:0] y_etop_store[0:9]; // one entry per block-column
reg [7:0]   y_eleft=128;
reg [3:0]   y_eblk_col=0;

loco_mb_enc_y_v3 #(.MB_W(32),.MB_H(32),.BUDGET(1028)) YENC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(y_enc_din),.s_axis_tvalid(y_enc_dv),.s_axis_tready(y_enc_ready),
    .top_ctx_valid(y_etop_v),.top_ctx_flat(y_etop_store[y_eblk_col]),
    .left_ctx_valid(y_eleft_v),.left_ctx_px(y_eleft),
    .m_axis_tdata(y_pkt_data),.m_axis_tvalid(y_pkt_valid),
    .m_axis_tready(1'b1),.m_axis_tlast(),
    .mb_done(y_enc_done),.mb_overflow(y_enc_ovf),.mb_comp_bytes(y_enc_bytes),
    .out_bottom_flat(y_enc_bottom),.out_right_px(y_enc_right)
);
always @(posedge clk) begin
    if(y_enc_done) begin
        y_etop_store[y_eblk_col]<=y_enc_bottom; y_eleft<=y_enc_right;
        if(y_eblk_col==9) begin y_eblk_col<=0;y_eleft_v<=0;y_etop_v<=1; end
        else begin y_eblk_col<=y_eblk_col+1;y_eleft_v<=1; end
    end
end

// ── Y decoder ─────────────────────────────────────────────────────────────────
reg  [7:0]  y_dec_din=0; reg y_dec_dv=0, y_dec_dlast=0;
wire        y_dec_dr;
wire [7:0]  y_dec_dout; wire y_dec_dv_out, y_dec_done;
wire [255:0] y_dec_bottom; wire [7:0] y_dec_right;

reg  y_dtop_v=0, y_dleft_v=0;
reg [255:0] y_dtop_store[0:9];
reg [7:0]   y_dleft=128;
reg [3:0]   y_dblk_col=0;

loco_mb_dec_v3 #(.MB_W(32),.MB_H(32),.BUDGET(1028)) YDEC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(y_dec_din),.s_axis_tvalid(y_dec_dv),
    .s_axis_tready(y_dec_dr),.s_axis_tlast(y_dec_dlast),
    .m_axis_tdata(y_dec_dout),.m_axis_tvalid(y_dec_dv_out),
    .m_axis_tready(1'b1),.m_axis_tlast(),.mb_done(y_dec_done),
    .top_ctx_valid(y_dtop_v),.top_ctx_flat(y_dtop_store[y_dblk_col]),
    .left_ctx_valid(y_dleft_v),.left_ctx_px(y_dleft),
    .out_bottom_flat(y_dec_bottom),.out_right_px(y_dec_right)
);
always @(posedge clk) begin
    if(y_dec_done) begin
        y_dtop_store[y_dblk_col]<=y_dec_bottom; y_dleft<=y_dec_right;
        if(y_dblk_col==9) begin y_dblk_col<=0;y_dleft_v<=0;y_dtop_v<=1; end
        else begin y_dblk_col<=y_dblk_col+1;y_dleft_v<=1; end
    end
end

// ── U encoder ─────────────────────────────────────────────────────────────────
reg  [7:0]  uenc_din; reg uenc_dv=0;
wire        uenc_ready;
wire [7:0]  u_pkt_data; wire u_pkt_valid;
wire        uenc_done; wire [10:0] uenc_bytes; wire uenc_ovf;
wire [127:0] u_enc_bottom; wire [7:0] u_enc_right;

reg  u_etop_v=0, u_eleft_v=0;
reg [127:0] u_etop_store[0:9];
reg [7:0]   u_eleft=128;
reg [3:0]   u_eblk_col=0;

loco_mb_enc_uv_v3 #(.MB_W(16),.MB_H(16),.BUDGET(260)) UENC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(uenc_din),.s_axis_tvalid(uenc_dv),.s_axis_tready(uenc_ready),
    .top_ctx_valid(u_etop_v),.top_ctx_flat(u_etop_store[u_eblk_col]),
    .left_ctx_valid(u_eleft_v),.left_ctx_px(u_eleft),
    .m_axis_tdata(u_pkt_data),.m_axis_tvalid(u_pkt_valid),
    .m_axis_tready(1'b1),.m_axis_tlast(),
    .mb_done(uenc_done),.mb_overflow(uenc_ovf),.mb_comp_bytes(uenc_bytes),
    .out_bottom_flat(u_enc_bottom),.out_right_px(u_enc_right)
);
always @(posedge clk) begin
    if(uenc_done) begin
        u_etop_store[u_eblk_col]<=u_enc_bottom; u_eleft<=u_enc_right;
        if(u_eblk_col==9) begin u_eblk_col<=0;u_eleft_v<=0;u_etop_v<=1; end
        else begin u_eblk_col<=u_eblk_col+1;u_eleft_v<=1; end
    end
end

// ── V encoder ─────────────────────────────────────────────────────────────────
reg  [7:0]  venc_din; reg venc_dv=0;
wire        venc_ready;
wire [7:0]  v_pkt_data; wire v_pkt_valid;
wire        venc_done; wire [10:0] venc_bytes; wire venc_ovf;
wire [127:0] v_enc_bottom; wire [7:0] v_enc_right;

reg  v_etop_v=0, v_eleft_v=0;
reg [127:0] v_etop_store[0:9];
reg [7:0]   v_eleft=128;
reg [3:0]   v_eblk_col=0;

loco_mb_enc_uv_v3 #(.MB_W(16),.MB_H(16),.BUDGET(260)) VENC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(venc_din),.s_axis_tvalid(venc_dv),.s_axis_tready(venc_ready),
    .top_ctx_valid(v_etop_v),.top_ctx_flat(v_etop_store[v_eblk_col]),
    .left_ctx_valid(v_eleft_v),.left_ctx_px(v_eleft),
    .m_axis_tdata(v_pkt_data),.m_axis_tvalid(v_pkt_valid),
    .m_axis_tready(1'b1),.m_axis_tlast(),
    .mb_done(venc_done),.mb_overflow(venc_ovf),.mb_comp_bytes(venc_bytes),
    .out_bottom_flat(v_enc_bottom),.out_right_px(v_enc_right)
);
always @(posedge clk) begin
    if(venc_done) begin
        v_etop_store[v_eblk_col]<=v_enc_bottom; v_eleft<=v_enc_right;
        if(v_eblk_col==9) begin v_eblk_col<=0;v_eleft_v<=0;v_etop_v<=1; end
        else begin v_eblk_col<=v_eblk_col+1;v_eleft_v<=1; end
    end
end

// ── UV decoder (shared, reset between U and V decode phases) ─────────────────
reg  [7:0]  uvdec_din=0; reg uvdec_dv=0, uvdec_dlast=0;
wire        uvdec_dr;
wire [7:0]  uvdec_dout; wire uvdec_dv_out, uvdec_done;
wire [127:0] uv_dec_bottom; wire [7:0] uv_dec_right;

reg  uv_dtop_v=0, uv_dleft_v=0;
reg [127:0] uv_dtop_store[0:9];
reg [7:0]   uv_dleft=128;
reg [3:0]   uv_dblk_col=0;

loco_mb_dec_v3 #(.MB_W(16),.MB_H(16),.BUDGET(260)) UVDEC(
    .clk(clk),.rst_n(rst_n),
    .s_axis_tdata(uvdec_din),.s_axis_tvalid(uvdec_dv),
    .s_axis_tready(uvdec_dr),.s_axis_tlast(uvdec_dlast),
    .m_axis_tdata(uvdec_dout),.m_axis_tvalid(uvdec_dv_out),
    .m_axis_tready(1'b1),.m_axis_tlast(),.mb_done(uvdec_done),
    .top_ctx_valid(uv_dtop_v),.top_ctx_flat(uv_dtop_store[uv_dblk_col]),
    .left_ctx_valid(uv_dleft_v),.left_ctx_px(uv_dleft),
    .out_bottom_flat(uv_dec_bottom),.out_right_px(uv_dec_right)
);
always @(posedge clk) begin
    if(uvdec_done) begin
        uv_dtop_store[uv_dblk_col]<=uv_dec_bottom; uv_dleft<=uv_dec_right;
        if(uv_dblk_col==9) begin uv_dblk_col<=0;uv_dleft_v<=0;uv_dtop_v<=1; end
        else begin uv_dblk_col<=uv_dblk_col+1;uv_dleft_v<=1; end
    end
end

// ── Frame stores ──────────────────────────────────────────────────────────────
reg [7:0] y_frame[0:60*1024-1];
reg [7:0] u_frame[0:60*256-1];
reg [7:0] v_frame[0:60*256-1];

// packet stores — all blocks buffered then decoded
reg [7:0] y_pkt_all[0:60*1028-1]; integer y_pkt_ptr=0;
reg [7:0] u_pkt_all[0:60*260-1];  integer u_pkt_ptr=0;
reg [7:0] v_pkt_all[0:60*260-1];  integer v_pkt_ptr=0;

reg [7:0] y_dec_all[0:60*1024-1]; integer y_dec_ptr=0;
reg [7:0] u_dec_all[0:60*256-1];
reg [7:0] v_dec_all[0:60*256-1];

always @(posedge clk) begin
    if(y_pkt_valid) begin y_pkt_all[y_pkt_ptr]=y_pkt_data; y_pkt_ptr=y_pkt_ptr+1; end
    if(u_pkt_valid) begin u_pkt_all[u_pkt_ptr]=u_pkt_data; u_pkt_ptr=u_pkt_ptr+1; end
    if(v_pkt_valid) begin v_pkt_all[v_pkt_ptr]=v_pkt_data; v_pkt_ptr=v_pkt_ptr+1; end
    if(y_dec_dv_out) begin y_dec_all[y_dec_ptr]=y_dec_dout; y_dec_ptr=y_dec_ptr+1; end
end

reg uv_is_u=1; integer uv_dec_ptr=0;
always @(posedge clk) begin
    if(uvdec_dv_out) begin
        if(uv_is_u) u_dec_all[uv_dec_ptr]=uvdec_dout;
        else        v_dec_all[uv_dec_ptr]=uvdec_dout;
        uv_dec_ptr=uv_dec_ptr+1;
    end
end

integer timeout,i,j,blk,blk_errors,cap_sz;
integer tot_ry=0,tot_cy=0,tot_ey=0,ovy=0,raty=0;
integer tot_ruv=0,tot_cuv=0,tot_euv=0,ovuv=0,ratuv=0;

initial begin
    $readmemh("frame_y_all.hex",y_frame);
    $readmemh("frame_u_all.hex",u_frame);
    $readmemh("frame_v_all.hex",v_frame);
    $display("==========================================================");
    $display("  FULL FRAME TEST v3  320x176  YUV 4:2:0  60 blocks each");
    $display("==========================================================");

    rst_n=0; repeat(8)@(posedge clk); rst_n=1; repeat(4)@(posedge clk);

    // ── ENCODE Y (all 60 blocks, context flows automatically) ─────────────────
    $display("--- ENCODING Y ---");
    for(i=0;i<60*1024;i=i+1) begin
        @(posedge clk); while(!y_enc_ready)@(posedge clk);
        y_enc_dv=1; y_enc_din=y_frame[i];
    end
    @(posedge clk); y_enc_dv=0;
    timeout=0;
    while(!y_enc_done&&timeout<5000000) begin @(posedge clk);timeout=timeout+1; end
    repeat(4)@(posedge clk);
    $display("  done: %0d bytes", y_pkt_ptr);

    // ── ENCODE U ──────────────────────────────────────────────────────────────
    $display("--- ENCODING U ---");
    for(i=0;i<60*256;i=i+1) begin
        @(posedge clk); while(!uenc_ready)@(posedge clk);
        uenc_dv=1; uenc_din=u_frame[i];
    end
    @(posedge clk); uenc_dv=0;
    timeout=0;
    while(!uenc_done&&timeout<2000000) begin @(posedge clk);timeout=timeout+1; end
    repeat(4)@(posedge clk);
    $display("  done: %0d bytes", u_pkt_ptr);

    // ── ENCODE V ──────────────────────────────────────────────────────────────
    $display("--- ENCODING V ---");
    for(i=0;i<60*256;i=i+1) begin
        @(posedge clk); while(!venc_ready)@(posedge clk);
        venc_dv=1; venc_din=v_frame[i];
    end
    @(posedge clk); venc_dv=0;
    timeout=0;
    while(!venc_done&&timeout<2000000) begin @(posedge clk);timeout=timeout+1; end
    repeat(4)@(posedge clk);
    $display("  done: %0d bytes", v_pkt_ptr);

    // ── DECODE Y ──────────────────────────────────────────────────────────────
    $display("--- Y CHANNEL ---");
    for(j=0;j<60*1028;j=j+1) begin
        @(posedge clk); while(!y_dec_dr)@(posedge clk);
        y_dec_dv=1; y_dec_din=y_pkt_all[j]; y_dec_dlast=(j==60*1028-1);
    end
    @(posedge clk); y_dec_dv=0; y_dec_dlast=0;
    timeout=0;
    while(!y_dec_done&&timeout<10000000) begin @(posedge clk);timeout=timeout+1; end
    repeat(10)@(posedge clk);
    for(blk=0;blk<60;blk=blk+1) begin
        blk_errors=0;
        for(i=0;i<1024;i=i+1)
            if(y_dec_all[blk*1024+i]!==y_frame[blk*1024+i]) blk_errors=blk_errors+1;
        cap_sz={y_pkt_all[blk*1028+1][1:0],y_pkt_all[blk*1028+2]};
        $display("Y  blk%02d (r%0d,c%0d)  %4dB  %3d%%  k=%0d  ovf=%0d  %s",
            blk,blk/10,blk%10,cap_sz,100-100*cap_sz/1024,
            y_pkt_all[blk*1028][6:5],y_pkt_all[blk*1028][7],
            blk_errors==0?"LOSSLESS":"ERRORS");
        tot_ry=tot_ry+1024; tot_cy=tot_cy+cap_sz;
        tot_ey=tot_ey+blk_errors; raty=raty+(100-100*cap_sz/1024);
        if(y_pkt_all[blk*1028][7]) ovy=ovy+1;
    end

    // ── DECODE U ──────────────────────────────────────────────────────────────
    $display("--- U CHANNEL ---");
    uv_is_u=1; uv_dec_ptr=0;
    // reset decoder context for UV
    rst_n=0; repeat(4)@(posedge clk); rst_n=1;
    uv_dtop_v=0; uv_dleft_v=0; uv_dblk_col=0;
    repeat(4)@(posedge clk);
    for(j=0;j<60*260;j=j+1) begin
        @(posedge clk); while(!uvdec_dr)@(posedge clk);
        uvdec_dv=1; uvdec_din=u_pkt_all[j]; uvdec_dlast=(j==60*260-1);
    end
    @(posedge clk); uvdec_dv=0; uvdec_dlast=0;
    timeout=0;
    while(!uvdec_done&&timeout<5000000) begin @(posedge clk);timeout=timeout+1; end
    repeat(10)@(posedge clk);
    for(blk=0;blk<60;blk=blk+1) begin
        blk_errors=0;
        for(i=0;i<256;i=i+1)
            if(u_dec_all[blk*256+i]!==u_frame[blk*256+i]) blk_errors=blk_errors+1;
        cap_sz={u_pkt_all[blk*260+1][1:0],u_pkt_all[blk*260+2]};
        $display("U  blk%02d (r%0d,c%0d)  %4dB  %3d%%  k=%0d  ovf=%0d  %s",
            blk,blk/10,blk%10,cap_sz,100-100*cap_sz/256,
            u_pkt_all[blk*260][6:5],u_pkt_all[blk*260][7],
            blk_errors==0?"LOSSLESS":"ERRORS");
        tot_ruv=tot_ruv+256; tot_cuv=tot_cuv+cap_sz;
        tot_euv=tot_euv+blk_errors; ratuv=ratuv+(100-100*cap_sz/256);
        if(u_pkt_all[blk*260][7]) ovuv=ovuv+1;
    end

    // ── DECODE V ──────────────────────────────────────────────────────────────
    $display("--- V CHANNEL ---");
    uv_is_u=0; uv_dec_ptr=0;
    rst_n=0; repeat(4)@(posedge clk); rst_n=1;
    uv_dtop_v=0; uv_dleft_v=0; uv_dblk_col=0;
    repeat(4)@(posedge clk);
    for(j=0;j<60*260;j=j+1) begin
        @(posedge clk); while(!uvdec_dr)@(posedge clk);
        uvdec_dv=1; uvdec_din=v_pkt_all[j]; uvdec_dlast=(j==60*260-1);
    end
    @(posedge clk); uvdec_dv=0; uvdec_dlast=0;
    timeout=0;
    while(!uvdec_done&&timeout<5000000) begin @(posedge clk);timeout=timeout+1; end
    repeat(10)@(posedge clk);
    for(blk=0;blk<60;blk=blk+1) begin
        blk_errors=0;
        for(i=0;i<256;i=i+1)
            if(v_dec_all[blk*256+i]!==v_frame[blk*256+i]) blk_errors=blk_errors+1;
        cap_sz={v_pkt_all[blk*260+1][1:0],v_pkt_all[blk*260+2]};
        $display("V  blk%02d (r%0d,c%0d)  %4dB  %3d%%  k=%0d  ovf=%0d  %s",
            blk,blk/10,blk%10,cap_sz,100-100*cap_sz/256,
            v_pkt_all[blk*260][6:5],v_pkt_all[blk*260][7],
            blk_errors==0?"LOSSLESS":"ERRORS");
        tot_ruv=tot_ruv+256; tot_cuv=tot_cuv+cap_sz;
        tot_euv=tot_euv+blk_errors; ratuv=ratuv+(100-100*cap_sz/256);
        if(v_pkt_all[blk*260][7]) ovuv=ovuv+1;
    end

    $display("\n==========================================================");
    $display("  SUMMARY");
    $display("==========================================================");
    $display("  Y  : avg %0d%%  raw %0dB -> %0dB  ovf %0d/60   errors %0d",
        raty/60,tot_ry,tot_cy,ovy,tot_ey);
    $display("  UV : avg %0d%%  raw %0dB -> %0dB  ovf %0d/120  errors %0d",
        ratuv/120,tot_ruv,tot_cuv,ovuv,tot_euv);
    $display("  Overall: %0d%%",
        100-100*(tot_cy+tot_cuv)/(tot_ry+tot_ruv));
    $display("  Total errors: %0d",tot_ey+tot_euv);
    $display("==========================================================");
    $finish;
end
endmodule
