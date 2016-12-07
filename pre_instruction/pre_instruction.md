# cReCompを用いた<br>FPGAモジュールのコンポーネント化
このチュートリアルでは簡単なFPGAモジュールのコンポーネント化を行います。

本チュートリアルで使用するFPGAモジュールは以下のようなものです。

```verilog
`timescale 1ns / 1ps
module adder(
	input clk,
	input rst,
	input [15:0] arg_x,
	input [15:0] arg_y,
	output [31:0] result
);

reg [31:0] result_reg;

always @(posedge clk) begin
	if (rst)
		result_reg <= 0;
	else
		result_reg <= arg_x + arg_y;
end

assign result = result_reg;

endmodule
```

このモジュールはinput信号である、`arg_x`、`arg_y`に入力値を与えることで、output信号`result`に加算した結果を返すという簡単な機能を持ちます。  

本チュートリアルではこの2つの信号にソフトウェアから入力値を与え、計算結果もソフトウェアにおいて受け取ることのできるようなコンポーネントを作成します。  

### 設定用ファイルのテンプレート生成

`pre_instruction/verilog/adder.v`があるかどうか確認します。

```
$ cd pre_instruction/verilog
$ ls
adder.v
```

cReCompを用いて、コンポーネント生成のための設定用ファイルを作ります。

- -p : 設定用ファイルの名前
- -u : コンポーネント化対象のFPGAモジュール(User logic)

`Generate config.py successfully`と出力されれば成功です。

```
$ crecomp -p config.py -u adder.v
Generate config.py successfully
```

生成された`config.py`は以下のようなものです。  
コードの詳細はコメントによって説明を追加しています。

```python
# -*- coding: utf-8 -*-
import crecomp.userlogic as ul
import crecomp.component as cp
import crecomp.verilog as vl
import crecomp.communication as com

# User logicに応じて生成されるクラス
# User logicのもつ入出力信号などの情報を持つ
class Adder(ul.Util):

	def __init__(self,uut):
		self.name = "adder"
		self.filepath = "adder.v"
		self.uut = uut
		self.ports =[
		vl.Input("clk", 1),
		vl.Input("rst", 1),
		vl.Input("arg_x", 16),
		vl.Input("arg_y", 16),
		vl.Output("result", 32)
		]
		self.assignlist = []

# コンポーネントの設定をするためのクラスのインスタンス
cp_adder = cp.Component("component_adder")

# ==================== for userlogic adder.v ====================
# User logicのクラスのインスタンス
adder = Adder("uut")

# adding signal for connection to user logic
cp_adder.add_input("clk",1)
cp_adder.add_input("rst",1)
cp_adder.add_input("arg_x",16)
cp_adder.add_input("arg_y",16)
cp_adder.add_output("result",32)

# cp_adder.add_reg("clk",1)
# cp_adder.add_reg("rst",1)
# cp_adder.add_reg("arg_x",16)
# cp_adder.add_reg("arg_y",16)
# cp_adder.add_wire("result",32)

# communication setting
fifo_32 = com.Xillybus_fifo(1,1,"1",32)
# fifo_32.assign(action = "rcv", sig = "signal_name")
# fifo_32.assign(action = "snd", sig = "signal_name")
cp_adder.add_com(fifo_32)

# fifo_8 = com.Xillybus_fifo(1,1,"1",8)

# fifo_8.assign(action = "rcv", sig = "signal_name")
# fifo_8.assign(action = "snd", sig = "signal_name")
# cp_adder.add_com(fifo_8)

# connection between software and user logic
adder.assign("clk","clk")
adder.assign("rst","rst")
adder.assign("arg_x","arg_x")
adder.assign("arg_y","arg_y")
adder.assign("result","result")

cp_adder.add_ul(adder)
cp_adder.ros_packaging()

cp_adder.componentize()

```
