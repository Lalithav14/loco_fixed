module quantiser(clk, rst_n, din, dv_in, qp, dout, dv_out);

input clk, rst_n, dv_in;
input [7:0] din;
input [4:0] qp;

output reg [7:0] dout;
output reg dv_out;

always @(posedge clk) begin
if(!rst_n) begin dout<=0; dv_out<=0; end
else begin
dv_out <= dv_in;
dout <= din >> qp;
end
end

endmodule
