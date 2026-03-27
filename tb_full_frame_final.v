module tb;

reg clk=0;
always #5 clk=~clk;

reg rst_n=0;

parameter QP=2;

integer i, blk;

reg [7:0] y_frame [0:51199];

reg [7:0] enc_din;
reg enc_dv;

wire [7:0] q_enc_din;
wire q_enc_dv;

wire [7:0] enc_out;
wire enc_valid;

integer bytes;
integer raw_bytes;

integer cycles=0;
integer blk_start;
integer blk_cycles;

integer total_cycles=0;

quantiser QY(clk,rst_n,enc_din,enc_dv,QP,q_enc_din,q_enc_dv);

loco_mb_enc ENC(
.clk(clk),
.rst_n(rst_n),
.s_axis_tdata(q_enc_din),
.s_axis_tvalid(q_enc_dv),
.m_axis_tdata(enc_out),
.m_axis_tvalid(enc_valid),
.m_axis_tlast()
);

always @(posedge clk) cycles++;

always @(posedge clk) begin
if(enc_valid) bytes = bytes + 1;
end

initial begin

$readmemh("frame_y_all.hex", y_frame);

#20 rst_n=1;

$display("==========================================================");
$display("  FULL FRAME TEST (OPTIMISED TB + QUANTISER)");
$display("==========================================================");

$display("\n--- Y CHANNEL ---");

for(blk=0; blk<50; blk=blk+1) begin

bytes = 0;
raw_bytes = 1024;

blk_start = cycles;

for(i=0;i<1024;i=i+1) begin
@(posedge clk);
enc_din = y_frame[blk*1024+i];
enc_dv = 1;
end

@(posedge clk);
enc_dv = 0;

wait(enc_valid);

#100;

blk_cycles = cycles - blk_start;
total_cycles = total_cycles + blk_cycles;

$display("Y blk%02d  %4dB  %2d%%  cycles=%0d",
blk,
bytes,
(100 - (bytes*100)/raw_bytes),
blk_cycles);

end

$display("\n==========================================================");
$display("Total cycles = %0d", total_cycles);
$display("==========================================================");

$finish;

end

endmodule
