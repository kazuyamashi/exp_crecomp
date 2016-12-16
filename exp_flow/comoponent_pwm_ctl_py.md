# component_pwm_ctl.pyの解説（コメントのみ）

以下の記述はcReCompによってすべて自動生成されます。

```python
#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
import rospy
from bridge import Xillybus

# cReCompが自動生成したコンポーネントで定義してあるメッセージタイプをインポート
from component_pwm_ctl.msg import Component_pwm_ctlMsg

# コンポーネントを扱うためのクラス
# ユーザロジックに応じてcReCompが自動生成する
class Component_pwm_ctl(object):
	# __init__() : インタンス生成時に実行される関数. コンストラクタのような役割.
	def __init__(self):
		# FPGAへの通信路へアクセスするためのインスタンス
		self.if_32 = Xillybus(32)
		self.if_32.open_dev_write()

		# ユーザロジックの信号のなかでと通信路と接続する設定をした信号が自動的に変数となる
		# input_* => input信号
		# output_* => output信号
		self.input_dir_in = 0
		self.input_para_in = 0

		# メッセージのインスタンス
		self.msg = Component_pwm_ctlMsg()
		# ユーザロジックから出力がある際に使用するPublisherのためのインスタンス
		self.pub = rospy.Publisher('component_pwm_ctl_output', Component_pwm_ctlMsg, queue_size=100)

	# 変数に格納した値を正しいbit配列でFPGAへ入力するためのメソッド
	def pack_32(self):
		data = 0
		data = data + (self.input_dir_in << 0)
		data = data + (self.input_para_in << 1)
		data = self.if_32.adjust(data, mode = 32)
		return data

	# コンポーネントが指定するTopicにおいてメッセージがPublishされたら起動するcall backメソッド
	def callback(self, data):

		self.input_dir_in = data.input_dir_in
		self.input_para_in = data.input_para_in

		# FPGAへのデータ入力
		self.if_32.write(self.pack_32())

		# please describe message data to publish
		# self.msg.your_msg_elem = self.if_32.read()

		# self.pub.publish(self.msg)

	# コンポーネントの起動する際に呼び出されるメソッド：メイン関数
	# 基本はSubscriberであり、メッセージの入力待ちをする
	def component_pwm_ctl(self):
		rospy.init_node('component_pwm_ctl', anonymous=True)
		rospy.Subscriber('component_pwm_ctl_input', Component_pwm_ctlMsg, self.callback)
		rospy.spin()

if __name__ == '__main__':
	component_pwm_ctl = Component_pwm_ctl()
	component_pwm_ctl.component_pwm_ctl()
```