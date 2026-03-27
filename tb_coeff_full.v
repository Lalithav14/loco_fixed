module tb;

reg clk=0;
always #5 clk=~clk;

reg rst_n=0;

integer i;

reg signed [7:0] coeff_mem [0:200000];

reg signed [7:0] in_data;
reg in_valid;

wire [7:0] mapped;
wire map_valid;

wire [7:0] enc_out;
wire enc_valid;

integer cycles=0;
integer bytes=0;
integer samples=0;

integer input_bits;
integer output_bits;

mapper MAP(
.clk(clk),
.rst_n(rst_n),
.in_data(in_data),
.in_valid(in_valid),
.out_data(mapped),
.out_valid(map_valid)
);

rice_encoder ENC(
.clk(clk),
.rst_n(rst_n),
.in_data(mapped),
.in_valid(map_valid),
.out_data(enc_out),
.out_valid(enc_valid)
);

always @(posedge clk) cycles = cycles + 1;

always @(posedge clk) begin
if(enc_valid) bytes = bytes + 1;
end

initial begin

$readmemh("coeffs.hex", coeff_mem);

#20 rst_n=1;

$display("====================================================");
$display(" COEFFICIENT PIPELINE TEST (QUANTISED INPUT)");
$display("====================================================");

bytes = 0;
samples = 0;

for(i=0;i<10000;i=i+1) begin
@(posedge clk);
in_data = coeff_mem[i];
in_valid = 1;
samples = samples + 1;
end

@(posedge clk);
in_valid = 0;

repeat(2000) @(posedge clk);

input_bits  = samples * 8;
output_bits = bytes * 8;

$display("----------------------------------------------------");
$display("Samples processed : %0d", samples);
$display("Total cycles      : %0d", cycles);
$display("Input bits        : %0d", input_bits);
$display("Output bits       : %0d", output_bits);
$display("Output bytes      : %0d", bytes);

if(output_bits != 0)
$display("Compression ratio : %0f", input_bits*1.0/output_bits);

$display("Compression (%%)  : %0d%%",
(100 - (output_bits*100)/input_bits));

$display("----------------------------------------------------");

$finish;

end

initial begin
#2000000;
$display("TIMEOUT - FORCED STOP");
$finish;
end

endmodule
