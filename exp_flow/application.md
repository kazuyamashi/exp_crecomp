# コンポーネントを用いて応用開発 <br>(Zedboard上のコンソールにおける作業)

cReCompによって生成したコンポーネントを用いて任意のモータ制御をおこなってみましょう。

以下のコマンドでROSのワークスペースにROSのパッケージの雛形を作成します。

```
$ cd ~/catkin_ws/src
$ catkin_create_pkg motor_ctl rospy
$ cd motor_ctl/
```

Pythonのプログラムを入れるためのディレクトリを作成し、そこに`motor_ctl.py`を作成します。

```
$ mkdir scripts; cd scripts
$ touch motor_ctl.py; chmod +x motor_ctl.py
```

Zedboard上のエディタで`motor_ctl.py`を開き、以下の内容をコピー＆ペーストします。  
(エディタはemacs、nano、viがインストールされています。)

```python
#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

import rospy
from component_pwm_ctl.msg import Component_pwm_ctlMsg

def motor_ctl():
	rospy.init_node('motor_ctl', anonymous=True)
	pub = rospy.Publisher('component_pwm_ctl_input', Component_pwm_ctlMsg, queue_size=100)
	msg = Component_pwm_ctlMsg()

	r = rospy.Rate(1)

	# msg.input_dir_in = 0
	# msg.input_para_in = 0

	while not rospy.is_shutdown():
		pub.publish(msg)
		r.sleep()

if __name__ == '__main__':
	motor_ctl()
```


以下のような機能仕様に合うように記述を追加して動作を確認してください。

- 各モータの制御をする関数を作成する（各パラメータは自由）
	- rotate_middle(msg) : 正回転、普通速度
	- rotate_high(msg)	 : 正回転、高速度
	- rotate_low(msg)		 : 正回転、低速度
	- rotate_reverse(msg)	 : 逆回転、普通速度
	- 各関数は，引数，戻り値共にComponent_pwm_ctlMsgのインスタンス(msg)にしてください．
- 実装した関数の動作確認をしましょう

**ヒント**

- ROS特有の記述に関してはコンポーネントの[テストプログラム解説](test_node_py.md)を参照
- Component_pwm_ctlMsg（cReCompが生成したメッセージタイプ）が持つ変数はinput_dir_inとinput_para_in
- Pythonの文法に関しては事前導入資料の[Pythonチュートリアル](../pre_instruction/md/python_tutorial.md)を参照

プログラムの記述が終わったら、以下のコマンドでビルドします。

```
$ cd ~/catkin_ws
$ catkin_make
```

パスの設定をして、プログラムを起動しましょう。

コンソール1

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun component_pwm_ctl component_pwm_ctl_node.py
```

コンソール2

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun motor_ctl motor_ctl.py
```

各プログラムは`Ctl + c`で停止します。
