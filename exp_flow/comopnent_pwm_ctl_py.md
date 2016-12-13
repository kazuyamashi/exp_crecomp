# component_pwm_ctl.pyの解説（コメントのみ）

以下の記述はcReCompによってすべて自動生成されます。

```python
#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
import rospy
from bridge import Xillybus
from component_pwm_ctl.msg import Component_pwm_ctlMsg

# コンポーネントを扱うためのクラス。

class Component_pwm_ctl(object):
	def __init__(self):
		self.if_32 = Xillybus(32)
		self.if_32.open_dev_write()

		self.input_dir_in = 0
		self.input_para_in = 0

		self.msg = Component_pwm_ctlMsg()
		self.pub = rospy.Publisher('component_pwm_ctl_output', Component_pwm_ctlMsg, queue_size=100)

	def pack_32(self):
		data = 0
		data = data + (self.input_dir_in << 0)
		data = data + (self.input_para_in << 1)
		data = self.if_32.adjust(data, mode = 32)
		return data

	def callback(self, data):
		self.input_dir_in = data.input_dir_in
		self.input_para_in = data.input_para_in

		self.if_32.write(self.pack_32())
		# please describe message data to publish
		# self.msg.your_msg_elem = self.if_32.read()

		# self.pub.publish(self.msg)

	def component_pwm_ctl(self):
		rospy.init_node('component_pwm_ctl', anonymous=True)
		rospy.Subscriber('component_pwm_ctl_input', Component_pwm_ctlMsg, self.callback)
		rospy.spin()

if __name__ == '__main__':
	component_pwm_ctl = Component_pwm_ctl()
	component_pwm_ctl.component_pwm_ctl()
```