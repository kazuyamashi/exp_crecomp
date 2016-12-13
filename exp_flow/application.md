# コンポーネントを用いて応用開発 @Zedboard上における作業

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

Zedboard状のエディタで`motor_ctl.py`を開き、以下の内容をコピー＆ペーストします。  
(エディタはemacs、nano、viがインストールされています。)

```
#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

import rospy
from component_pwm_ctl.msg import Component_pwm_ctlMsg

if __name__ == '__main__':

	rospy.init_node('motor_ctl', anonymous=True)
	pub = rospy.Publisher('component_pwm_ctl_input', Component_pwm_ctlMsg, queue_size=100)
	msg = Component_pwm_ctlMsg()

	r = rospy.Rate(1)

	# msg.input_dir_in = 0
	# msg.input_para_in = 0

	while not rospy.is_shutdown():
		pub.publish(msg)
		r.sleep()
```


以下のような機能仕様プログラムを作成して動作を確認してさい。



