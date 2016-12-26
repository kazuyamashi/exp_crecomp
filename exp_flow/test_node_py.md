# test_node.py(編集済み)の解説

```python
#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
import rospy
from component_pwm_ctl.msg import Component_pwm_ctlMsg

def component_pwm_ctl_pub():
		rospy.init_node('component_pwm_ctl_pub', anonymous=True)
		pub = rospy.Publisher('component_pwm_ctl_input', Component_pwm_ctlMsg, queue_size=100)

		r = rospy.Rate(1)
		msg = Component_pwm_ctlMsg()

		msg.input_dir_in = 1
		msg.input_para_in = 12000

		while not rospy.is_shutdown():
				pub.publish(msg)
				r.sleep()


# ========== 今回は使用しません ==========
def callback(data):
		# please desicribe your code
		pass

def component_pwm_ctl_sub(self):
		rospy.init_node('component_pwm_ctl_sub', anonymous=True)
		rospy.Subscriber('component_pwm_ctl_output', Component_pwm_cack)
		rospy.spin()
# ========== 今回は使用しません ==========

if __name__ == '__main__':
	component_pwm_ctl_pub()
```



```python
import rospy
from component_pwm_ctl.msg import Component_pwm_ctlMsg
```

このプログラムに必要なPythonのモジュールしています。  

- rospy
	- ROSのプログラミングの際に必須なモジュール
- component_pwm_ctl
	- cReCompによって自動生成されたコンポーネントのモジュール。自動生成されたメッセージの方を使用するためにインポートしている。


```python
def component_pwm_ctl_pub():
		rospy.init_node('component_pwm_ctl_pub', anonymous=True)
		pub = rospy.Publisher('component_pwm_ctl_input', Component_pwm_ctlMsg, queue_size=100)

		r = rospy.Rate(1)
		msg = Component_pwm_ctlMsg()

		msg.input_dir_in = 1
		msg.input_para_in = 12000

		while not rospy.is_shutdown():
				pub.publish(msg)
				r.sleep()
```

コンポーネントへメッセージをPublishするための関数記述です。

- `rospy.init_node('component_pwm_ctl_pub', anonymous=True)`
	- このROSアプリケーションが`component_pwm_ctl_pub`という名前であることを定義します。
- `pub = rospy.Publisher('component_pwm_ctl_input', Component_pwm_ctlMsg, queue_size=100)`
	- Publishするための設定をします。`Publisher()`の引数は左からPublishするための**トピック名**、**メッセージタイプ**、**キューサイズ**です。したがって、この記述で、component_pwm_ctl_inputという名前のトピックにComponent_pwm_ctlMsg型のメッセージをPublishするということになります。
- `r = rospy.Rate(1)`
	- 引数はループ処理（スリープ）を何回するかの周波数を与えます。上記の記述では**引数に**1**を与えていることで、1秒に1回メッセージをPublishしています。**
	- 例えば，引数を0.5にすれば，2秒に1回Publishすることになります．
- `msg = Component_pwm_ctlMsg()`
	- cReCompによって自動生成された`Component_pwm_ctlMsg`型のメッセージのインスタンスを作成しています。
	- **msg.input_dir_in** : Component_pwm_ctlMsgがもつ、ユーザロジックの信号`dir_in`へのデータ入力用の変数
	- **msg.input_para_in** : Component_pwm_ctlMsgがもつ、ユーザロジックの信号`para_in`へのデータ入力用の変数
- `pub.publish(msg)`
	- メッセージをPublishするためのメソッド


```python
if __name__ == '__main__':
	component_pwm_ctl_pub()
```

Python特有の記述で、メイン実行をする部分です。