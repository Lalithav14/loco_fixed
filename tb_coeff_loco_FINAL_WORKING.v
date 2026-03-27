module tb;

reg clk=0;
always #5 clk=~clk;

reg rst_n=0;

integer i, blk;

reg [7:0] coeff_mem [0:200000];

reg [7:0] enc_din;
reg enc_dv;
reg enc_last;

wire [7:0] enc_out;
wire enc_valid;

integer cycles=0;
integer total_bytes=0;

integer blk_bytes;
integer blk_cycles_start;
integer blk_cycles;

loco_mb_enc ENC(
.clk(clk),
.rst_n(rst_n),
.s_axis_tdata(enc_din),
.s_axis_tvalid(enc_dv),
.s_axis_tlast(enc_last),
.m_axis_tdata(enc_out),
.m_axis_tvalid(enc_valid),
.m_axis_tlast()
);

always @(posedge clk) cycles++;

always @(posedge clk) begin
if(enc_valid) begin
    total_bytes = total_bytes + 1;
    blk_bytes   = blk_bytes + 1;
end
end

initial begin

$readmemh("coeffs.hex", coeff_mem);

#20 rst_n=1;

$display("============================================================");
$display("        COEFFICIENT → LOCO ENCODER ANALYSIS");
$display("============================================================");

for(blk=0; blk<5; blk=blk+1) begin

blk_bytes = 0;
blk_cycles_start = cycles;

$display("\n--- BLOCK %0d ---", blk);

for(i=0;i<1024;i=i+1) begin
@(posedge clk);
enc_din  = coeff_mem[blk*1024 + i];
enc_dv   = 1;
enc_last = (i == 1023);

// 🔴 CRITICAL FIX: insert gap every cycle
@(posedge clk);
enc_dv   = 0;
enc_last = 0;
end

repeat(3000) @(posedge clk);

blk_cycles = cycles - blk_cycles_start;

$display("Output bytes       : %0d", blk_bytes);
$display("Block cycles       : %0d", blk_cycles);

if(blk_bytes != 0)
$display("Compression (%%)   : %0d%%",
(100 - (blk_bytes*8*100)/(1024*8)));
else
$display("Compression        : NO OUTPUT");

$display("--------------------------------------------");

end

$display("\nTOTAL BYTES: %0d", total_bytes);

$finish;

end

initial begin
#3000000;
$display("TIMEOUT");
$finish;
end

endmodule
