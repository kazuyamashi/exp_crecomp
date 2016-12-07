# 被験者実験手順指導書

# PWM回路のコンポーネント化

これらより、`~/exp_dir`のことをワークスペースと呼びます。
ワークスペースに`pwm_ctl.v`と`xillinux-eval-zedboard-1.3c`があるか確認してください。

```
$ ls ~/exp_dir
pwm_ctl.v xillinux-eval-zedboard-1.3c
```

cReCompが動作するか確認します。

```
$ crecomp -h
Usage: crecomp [option] [file name] [-u user logic path]+

Options:
  -h, --help            show this help message and exit
  -u USERLOGIC, --userlogic=USERLOGIC
                        specifier your user logic name
  -p PYTHON_TEMPLATENAME, --python_template=PYTHON_TEMPLATENAME
                        specifier for template name
  -s SCRP_TEMPLATENAME, --scrp_template=SCRP_TEMPLATENAME
                        specifier for template name
  -b SCRP_PATH, --build=SCRP_PATH
                        specifier target scrp file to build for componentize
  -t TEST_PATH, --test=TEST_PATH
                        generate testbench of target user logic
```

今回、作成するシステムの機能使用は以下のようなものです。

- 機能1：モータをPWMによって制御
- 機能2：FPGA上の回路へ、ソフトウェアからパラメータを渡せるようにする
- 機能3：ROSのメッセージによって回路へのパラメータを渡せるようにする

<!-- 図 -->

図のようにコンポーネントはソフトウェアとハードウェアの混合システムとなっています。  
特に、cReCompではハードウェアインターフェイスとソフトウェアインターフェイスを設定に応じて自動生成することができ、  
開発者の皆さんはVerilog HDLの記述やデータをやり取りするためのソフトウェア記述はいりません。  
はじめに、以下のコマンドで設定ファイルを作成します。
`Generate config.py successfully`が出力されたら成功です。

```
$ crecomp -p config.py -u pwm_ctl.v
Generate config.py successfully
```

作成に成功すると`~/exp_dir/config.py`ができ、以下のような内容になっているはずです。

```python
# -*- coding: utf-8 -*-

import crecomp.userlogic as ul
import crecomp.component as cp
import crecomp.verilog as vl
import crecomp.communication as com


class Pwm_ctl(ul.Util):

	def __init__(self,uut):
		self.name = "pwm_ctl"
		self.filepath = "pwm_ctl.v"
		self.uut = uut
		self.ports =[
		vl.Input("clk", 1),
		vl.Input("rst", 1),
		vl.Input("para_in", 15),
		vl.Input("dir_in", 1),
		vl.Output("dir_out", 1),
		vl.Output("en_out", 1)
		]
		self.assignlist = []

cp_pwm_ctl = cp.Component("component_pwm_ctl")


# ==================== for userlogic pwm_ctl.v ====================
pwm_ctl = Pwm_ctl("uut")

# adding signal for connection to user logic
cp_pwm_ctl.add_input("clk",1)
cp_pwm_ctl.add_input("rst",1)
cp_pwm_ctl.add_input("para_in",15)
cp_pwm_ctl.add_input("dir_in",1)
cp_pwm_ctl.add_output("dir_out",1)
cp_pwm_ctl.add_output("en_out",1)

# cp_pwm_ctl.add_reg("clk",1)
# cp_pwm_ctl.add_reg("rst",1)
# cp_pwm_ctl.add_reg("para_in",15)
# cp_pwm_ctl.add_reg("dir_in",1)
# cp_pwm_ctl.add_wire("dir_out",1)
# cp_pwm_ctl.add_wire("en_out",1)

# communication setting
fifo_32 = com.Xillybus_fifo(1,1,"1",32)
# fifo_32.assign(action = "rcv", sig = "signal_name")
# fifo_32.assign(action = "snd", sig = "signal_name")
cp_pwm_ctl.add_com(fifo_32)

# fifo_8 = com.Xillybus_fifo(1,1,"1",8)

# fifo_8.assign(action = "rcv", sig = "signal_name")
# fifo_8.assign(action = "snd", sig = "signal_name")
# cp_pwm_ctl.add_com(fifo_8)

# connection between software and user logic
pwm_ctl.assign("clk","clk")
pwm_ctl.assign("rst","rst")
pwm_ctl.assign("para_in","para_in")
pwm_ctl.assign("dir_in","dir_in")
pwm_ctl.assign("dir_out","dir_out")
pwm_ctl.assign("en_out","en_out")

cp_pwm_ctl.add_ul(pwm_ctl)
cp_pwm_ctl.ros_packaging()

cp_pwm_ctl.componentize()
```


編集する箇所を解説していきます。まず、コンポーネントのハードウェアインターフェイスに必要な信号を追加します。  
特に、**モータへの出力に必要なOutput信号**と**ソフトウェアからパラメータをもらうためのregister信号**が必要となります。

```diff
# adding signal for connection to user logic
- cp_pwm_ctl.add_input("clk",1)
- cp_pwm_ctl.add_input("rst",1)
- cp_pwm_ctl.add_input("para_in",15)
- cp_pwm_ctl.add_input("dir_in",1)
cp_pwm_ctl.add_output("dir_out",1)
cp_pwm_ctl.add_output("en_out",1)

- # cp_pwm_ctl.add_reg("clk",1)
- # cp_pwm_ctl.add_reg("rst",1)
+ cp_pwm_ctl.add_reg("para_in",15)	# コメント解除
+ cp_pwm_ctl.add_reg("dir_in",1)	# コメント解除
- # cp_pwm_ctl.add_wire("dir_out",1)
- # cp_pwm_ctl.add_wire("en_out",1)
```

次に、PWM回路とソフトウェアを接続するための設定をします。
モータ回路への入力は**para_inの15bit**と**dir_inの1bit**です。ソフトウェアとハードウェアの間の通信路は32bitですので、パラメータを1
回入力(SW→HW)するには1回のデータ入力のみで良い、ということになります。  
また、PWM回路からソフトウェア側へのデータ送信は必要ありません。

そこで`set_snd_cycle()`を使用することでHW→SWへのデータ送信回数を設定できます。

また、PWM回路へソフトウェアから受信したデータを渡さなければなりません。  
したがって、前の編集で追加したレジスタ`para_in`と`dir_in`をデータ受信用の信号として設定します。

したがって、設定すべき内容は以下のようになります。

- ソフトウェアからハードウェアへデータを受信する回数：1回
- ハードウェアからソフトウェアへデータを送信する回数：0回
- ソフトウェアからデータを受け取るため、以下の2つの信号を入力用通信路へと接続
	- dir_in
	- para_in

```diff
# communication setting
fifo_32 = com.Xillybus_fifo(1,1,"1",32)
+ fifo_32.set_snd_cycle(0)
- # fifo_32.assign(action = "rcv", sig = "signal_name")
+ fifo_32.assign(action = "rcv", sig = "dir_in")
+ fifo_32.assign(action = "rcv", sig = "para_in")
# fifo_32.assign(action = "snd", sig = "signal_name")
cp_pwm_ctl.add_com(fifo_32)
```

以上の設定記述を終えると、`config.py`は以下のようになっているはずです。

```python
# -*- coding: utf-8 -*-

import crecomp.userlogic as ul
import crecomp.component as cp
import crecomp.verilog as vl
import crecomp.communication as com


class Pwm_ctl(ul.Util):

	def __init__(self,uut):
		self.name = "pwm_ctl"
		self.filepath = "pwm_ctl.v"
		self.uut = uut
		self.ports =[
		vl.Input("clk", 1),
		vl.Input("rst", 1),
		vl.Input("para_in", 15),
		vl.Input("dir_in", 1),
		vl.Output("dir_out", 1),
		vl.Output("en_out", 1)
		]
		self.assignlist = []

cp_pwm_ctl = cp.Component("component_pwm_ctl")

# ==================== for userlogic pwm_ctl.v ====================
pwm_ctl = Pwm_ctl("uut")

# adding signal for connection to user logic

cp_pwm_ctl.add_output("dir_out",1)
cp_pwm_ctl.add_output("en_out",1)

cp_pwm_ctl.add_reg("para_in",15)
cp_pwm_ctl.add_reg("dir_in",1)


# communication setting
fifo_32 = com.Xillybus_fifo(1,1,"1",32)
fifo_32.set_snd_cycle(0)
fifo_32.assign(action = "rcv", sig = "dir_in")
fifo_32.assign(action = "rcv", sig = "para_in")
# fifo_32.assign(action = "snd", sig = "signal_name")
cp_pwm_ctl.add_com(fifo_32)

# fifo_8 = com.Xillybus_fifo(1,1,"1",8)

# fifo_8.assign(action = "rcv", sig = "signal_name")
# fifo_8.assign(action = "snd", sig = "signal_name")
# cp_pwm_ctl.add_com(fifo_8)

# connection between software and user logic
pwm_ctl.assign("clk","clk")
pwm_ctl.assign("rst","rst")
pwm_ctl.assign("para_in","para_in")
pwm_ctl.assign("dir_in","dir_in")
pwm_ctl.assign("dir_out","dir_out")
pwm_ctl.assign("en_out","en_out")

cp_pwm_ctl.add_ul(pwm_ctl)
cp_pwm_ctl.ros_packaging()

cp_pwm_ctl.componentize()
```

コンポーネント化を実行します。以下のコマンドを実行してください。  
`Generate component successfully`が出力されたら成功です。

```
$ python config.py
Generate component successfully
```

生成されたものを確認してみましょう。

```
$ cd component_pwm_ctl ; ls
hardware  software
```

生成物は以下のようなディレクトリ構造になっています。

```
component_pwm_ctl/
|--hardware/
|	|--component_pwm_ctl.v
|	|--pwm_ctl.v
|--software/
	|--bridge.py
	|--component_pwm_ctl.py
	|--ros_package/
		|--component_pwm_ctl/
```


```c
//for xdc file
set_property -dict "PACKAGE_PIN W12 IOSTANDARD LVCMOS33" [get_ports "dir_out"]
set_property -dict "PACKAGE_PIN W11 IOSTANDARD LVCMOS33" [get_ports "en_out"]
set_property -dict "PACKAGE_PIN V10 IOSTANDARD LVCMOS33" [get_ports "dummy[0]"]
set_property -dict "PACKAGE_PIN W8 	IOSTANDARD LVCMOS33" [get_ports "dummy[1]"]
set_property -dict "PACKAGE_PIN V12 IOSTANDARD LVCMOS33" [get_ports "dummy[2]"]
set_property -dict "PACKAGE_PIN W10 IOSTANDARD LVCMOS33" [get_ports "dummy[3]"]
set_property -dict "PACKAGE_PIN V9 	IOSTANDARD LVCMOS33" [get_ports "dummy[4]"]
set_property -dict "PACKAGE_PIN V8 	IOSTANDARD LVCMOS33" [get_ports "dummy[5]"]


set_property -dict "PACKAGE_PIN M20 IOSTANDARD LVCMOS33" [get_ports "PS_GPIO[32]"]
set_property -dict "PACKAGE_PIN M19 IOSTANDARD LVCMOS33" [get_ports "PS_GPIO[33]"]
set_property -dict "PACKAGE_PIN N20 IOSTANDARD LVCMOS33" [get_ports "PS_GPIO[34]"]
set_property -dict "PACKAGE_PIN N19 IOSTANDARD LVCMOS33" [get_ports "PS_GPIO[35]"]
set_property -dict "PACKAGE_PIN P18 IOSTANDARD LVCMOS33" [get_ports "PS_GPIO[36]"]
set_property -dict "PACKAGE_PIN P17 IOSTANDARD LVCMOS33" [get_ports "PS_GPIO[37]"]
set_property -dict "PACKAGE_PIN P22 IOSTANDARD LVCMOS33" [get_ports "PS_GPIO[38]"]
set_property -dict "PACKAGE_PIN N22 IOSTANDARD LVCMOS33" [get_ports "PS_GPIO[39]"]
```

```verilog
//for top module
output dir_out,
output en_out,
input [5:0] dummy
```