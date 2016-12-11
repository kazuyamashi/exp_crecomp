`timescale 1ns / 1ps
module adder(
	input [15:0] arg_x,
	input [15:0] arg_y,
	output [31:0] result
);

assign result = arg_x + arg_y;

endmodule