# FPGAとCPUをつなぐインターフェイスの記述体験

cReCompを用いずに、ユーザロジックとソフトウェアとで、データをやり取りする場合、ユーザロジック(FPGAの回路記述)へのデータアクセスするためのインターフェイス記述が必要です。  
また、ROSシステムへ導入するためのROSの記述も必要となります。すなわち、具体的には以下のような記述（作業工程）があります。

1. ハードウェアとソフトウェアがデータ通信を行うための通信路制御記述
- 通信路へアクセスするためのラッパソフトウェア(ライブラリ)の作成
- ラッパソフトウェアを用いて、ROSのプログラムを作成

また、実際には記述の他に設計・デバッグが存在します。

本作業では、FPGAのユーザロジックをROSシステムへ導入する際に、実際に記述する必要があるハードウェアとソフトウェアに関する記述を体験していただき、時間計測を行います。以下の2つのファイルを記述してください。

- 各記述の機能は今回作成したコンポーネントとほぼ同等の機能を持ちます。
- 自動生成する際に記述される、冗長的な表現・機能は削除し、最低限の記述になっています。
- 各記述はPC上でおこなってください。

## 通信路の制御記述(Verilog HDL) ファイル名：communication.v

```verilog
`timescale 1ns / 1ps

module component_pwm_ctl(
input [32:0] din_32,
input wr_en_32,
input rd_en_32,
input clk,
input rst,

output dir_out,
output en_out,
output [32:0] dout_32,
output full_32,
output empty_32
);

reg [15:0] para_in;
reg dir_in;
reg [3:0] state_32;
reg rcv_en_32;
reg snd_en_32;

wire [32:0] rcv_data_32;
wire data_empty_32;
wire [32:0] snd_data_32;
wire data_full_32;

pwm_ctl uut(
.clk(clk),
.rst(rst),
.para_in(para_in),
.dir_in(dir_in),
.dir_out(dir_out),
.en_out(en_out)
);

parameter INIT_32 = 0,
		IDLE_32 = 1,
		READY_RCV_32 = 2,
		RCV_DATA_32_0 = 3,
		RCV_END_32 = 4,
		POSE_32 = 5,
		CYCLE_END_32 = 6;

fifo_32x512 input_fifo_32(
	.clk(clk),
	.srst(rst),
	.din(din_32),
	.wr_en(wr_en_32),
	.full(full_32),
	.dout(rcv_data_32),
	.rd_en(rcv_en_32),
	.empty(data_empty_32)
	);

fifo_32x512 output_fifo_32(
	.clk(clk),
	.srst(rst),
	.din(snd_data_32),
	.wr_en(snd_en_32),
	.full(data_full_32),
	.dout(dout_32),
	.rd_en(rd_en_32),
	.empty(empty_32)
	);

always @(posedge clk)begin
	if(rst)
		state_32 <= 0;
	else
		case (state_32)
			INIT_32: 		state_32 <= IDLE_32;
			IDLE_32:		state_32 <= READY_RCV_32;
			READY_RCV_32:	if(data_empty_32 == 0)	state_32 <= RCV_DATA_32_0;
			RCV_DATA_32_0:	state_32 <=RCV_END_32;
			RCV_END_32:		state_32 <= POSE_32;
			POSE_32: if(1)	state_32 <= CYCLE_END_32;
			CYCLE_END_32:	state_32 <= IDLE_32;
			default: state_32 <= INIT_32;
		endcase
end

always @(posedge clk)begin
	if(rst)begin
		dir_in <= 0;
		para_in <= 0;
	end
	else if (rcv_en_32)begin
		dir_in <= rcv_data_32[0:0];
		para_in <= rcv_data_32[15:1];
	end
	else begin
	end
end

always @(posedge clk)begin
	if(rst)begin
		snd_en_32 <= 0;
		rcv_en_32 <= 0;
	end
	else case (state_32)
		READY_RCV_32: if(data_empty_32 == 0)	rcv_en_32 <= 1;
		CYCLE_END_32: rcv_en_32 <= 0;
	endcase
end
endmodule
```

## 通信路へアクセスするためのラッパの記述とROSのプログラム(Python)<br>ファイル名：pwm_ctl.py

```python
# -*- coding: utf-8 -*-
import struct
import os

class Component_pwm_ctl(object):
	def __init__(self):

		self.fw = os.open("/dev/xillybus_write_32", os.O_WRONLY)

		self.input_dir_in = 0
		self.input_para_in = 0

		self.msg = Component_pwm_ctlMsg()

	def pack_32(self):
		data = 0
		data = data + (self.input_dir_in << 0)
		data = data + (self.input_para_in << 1)
		data = struct.pack("I",data)
		return data

	def callback(self, data):
		self.input_dir_in = data.input_dir_in
		self.input_para_in = data.input_para_in

		os.write(self.fw ,self.pack_32())

	def component_pwm_ctl(self):
		rospy.init_node('component_pwm_ctl', anonymous=True)
		rospy.Subscriber('component_pwm_ctl_input', Component_pwm_ctlMsg, self.callback)
		rospy.spin()

if __name__ == '__main__':
	component_pwm_ctl = Component_pwm_ctl()
	component_pwm_ctl.component_pwm_ctl()
	os.close(component_pwm_ctl.fw)

```

[Topへ戻る](readme.md)