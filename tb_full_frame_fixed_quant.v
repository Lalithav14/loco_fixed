module tb;

reg clk=0;
always #5 clk=~clk;

reg rst_n=0;

parameter QP = 2;

integer i, blk;

reg [7:0] y_frame [0:51199];
reg [7:0] u_frame [0:25599];
reg [7:0] v_frame [0:25599];

reg [7:0] enc_din;
reg enc_dv;

wire [7:0] q_enc_din;
wire q_enc_dv;

reg [7:0] uenc_din;
reg uenc_dv;

wire [7:0] q_uenc_din;
wire q_uenc_dv;

reg [7:0] venc_din;
reg venc_dv;

wire [7:0] q_venc_din;
wire q_venc_dv;

integer cycles=0;
integer blk_start;
integer blk_cycles;

quantiser QY(clk,rst_n,enc_din,enc_dv,QP,q_enc_din,q_enc_dv);
quantiser QU(clk,rst_n,uenc_din,uenc_dv,QP,q_uenc_din,q_uenc_dv);
quantiser QV(clk,rst_n,venc_din,venc_dv,QP,q_venc_din,q_venc_dv);

loco_mb_enc ENC(
.clk(clk),
.rst_n(rst_n),
.s_axis_tdata(q_enc_din),
.s_axis_tvalid(q_enc_dv),
.m_axis_tdata(),
.m_axis_tvalid(),
.m_axis_tlast()
);

loco_mb_dec DEC(
.clk(clk),
.rst_n(rst_n),
.s_axis_tdata(),
.s_axis_tvalid(),
.m_axis_tdata(),
.m_axis_tvalid()
);

always @(posedge clk) cycles = cycles + 1;

initial begin

$readmemh("frame_y_all.hex", y_frame);
$readmemh("frame_u_all.hex", u_frame);
$readmemh("frame_v_all.hex", v_frame);

#20 rst_n=1;

$display("==========================================================");
$display("  FULL FRAME TEST (WITH QUANTISER)");
$display("==========================================================");

$display("\n--- Y CHANNEL ---");

for(blk=0; blk<50; blk=blk+1) begin

blk_start = cycles;

for(i=0;i<1024;i=i+1) begin
@(posedge clk);
enc_din = y_frame[blk*1024+i];
enc_dv = 1;
end

@(posedge clk);
enc_dv = 0;

blk_cycles = cycles - blk_start;

$display("Y blk%02d cycles=%0d",blk,blk_cycles);

end

$display("\n--- U CHANNEL ---");

for(blk=0; blk<50; blk=blk+1) begin

blk_start = cycles;

for(i=0;i<256;i=i+1) begin
@(posedge clk);
uenc_din = u_frame[blk*256+i];
uenc_dv = 1;
end

@(posedge clk);
uenc_dv = 0;

blk_cycles = cycles - blk_start;

$display("U blk%02d cycles=%0d",blk,blk_cycles);

end

$display("\n--- V CHANNEL ---");

for(blk=0; blk<50; blk=blk+1) begin

blk_start = cycles;

for(i=0;i<256;i=i+1) begin
@(posedge clk);
venc_din = v_frame[blk*256+i];
venc_dv = 1;
end

@(posedge clk);
venc_dv = 0;

blk_cycles = cycles - blk_start;

$display("V blk%02d cycles=%0d",blk,blk_cycles);

end

$display("\nTotal cycles = %0d", cycles);

$finish;

end

endmodule
