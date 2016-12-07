`timescale 1ns / 1ps
module adder(
	input clk,
	input rst,
	input [15:0] arg_x,
	input [15:0] arg_y,
	output reg [31:0] result
);

reg [31:0] result_reg;

// always @(posedge clk) begin
// 	if (rst)
// 		result_reg <= 0;
// 	else
// 		result_reg <= arg_x + arg_y;
// end

assign result = arg_x + arg_y;

endmodule