module tb;

reg clk=0;
always #5 clk=~clk;

reg rst_n=0;

integer i, blk;

reg [7:0] coeff_mem [0:200000];

reg [7:0] enc_din;
reg enc_dv;

wire [7:0] enc_out;
wire enc_valid;

integer cycles=0;
integer total_bytes=0;

integer blk_bytes;
integer blk_cycles_start;
integer blk_cycles;

integer total_input_bits;
integer total_output_bits;

loco_mb_enc ENC(
.clk(clk),
.rst_n(rst_n),
.s_axis_tdata(enc_din),
.s_axis_tvalid(enc_dv),
.m_axis_tdata(enc_out),
.m_axis_tvalid(enc_valid),
.m_axis_tlast()
);

always @(posedge clk) cycles = cycles + 1;

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

total_bytes = 0;
total_input_bits = 0;

for(blk=0; blk<5; blk=blk+1) begin

blk_bytes = 0;
blk_cycles_start = cycles;

$display("\n--- BLOCK %0d ---", blk);

for(i=0;i<1024;i=i+1) begin
@(posedge clk);
enc_din = coeff_mem[blk*1024 + i];
enc_dv  = 1;
end

@(posedge clk);
enc_dv = 0;

repeat(2000) @(posedge clk);

blk_cycles = cycles - blk_cycles_start;

total_input_bits = total_input_bits + (1024*8);

$display("Input samples      : 1024");
$display("Input bits         : %0d", 1024*8);
$display("Output bytes       : %0d", blk_bytes);
$display("Output bits        : %0d", blk_bytes*8);

if(blk_bytes != 0) begin
$display("Compression ratio  : %0f",
(1024.0*8)/(blk_bytes*8));

$display("Compression (%%)   : %0d%%",
(100 - (blk_bytes*8*100)/(1024*8)));
end else begin
$display("Compression        : NO OUTPUT");
end

$display("Block cycles       : %0d", blk_cycles);

if(blk_cycles != 0)
$display("Throughput         : %0f pixels/cycle",
1024.0/blk_cycles);

$display("--------------------------------------------");

end

total_output_bits = total_bytes * 8;

$display("\n============================================================");
$display("                    FINAL SUMMARY");
$display("============================================================");

$display("Total cycles       : %0d", cycles);
$display("Total output bytes : %0d", total_bytes);
$display("Total input bits   : %0d", total_input_bits);
$display("Total output bits  : %0d", total_output_bits);

if(total_output_bits != 0) begin
$display("Overall ratio      : %0f",
total_input_bits*1.0/total_output_bits);

$display("Overall reduction  : %0d%%",
(100 - (total_output_bits*100)/total_input_bits));
end

$display("============================================================");

$finish;

end

initial begin
#2000000;
$display("TIMEOUT - FORCED STOP");
$finish;
end

endmodule
