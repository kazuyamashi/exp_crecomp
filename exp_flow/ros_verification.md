# ROS上でコンポーネントの動作確認する

SDカードをZedbardに挿入し、ZedbaordとPCをUSBケーブルで接続します。
接続したら、起動します。  

以下のコマンドでZedboardへシリアル接続します。

```
$ sudo screen /dev/ttyACM0 115200
```

Enterキーを数回押すと、以下のような出力が得られます。

```
zynq-boot > 
zynq-boot > 
zynq-boot > 
```

これはブートローダのコンソールです。**boot**コマンドでLinuxを起動します。

```
zynq-boot > boot
```

以下のような出力されたら起動成功です。

```
Last login: Thu Jan  1 00:00:14 UTC 1970 on ttyPS0
Welcome to Ubuntu 14.04 LTS (GNU/Linux 3.18.0-xilinx-46110-gd627f5d armv7l)

 * Documentation:  https://help.ubuntu.com/
root@ubuntu-armhf:~# 
```

<font color="red">Zedboard上における作業</font>  
Zedboardの時間の設定をします。  
書式：`date MMDDhhmm2016 日付時間年`

```
# date MMDDhhmm2016
```

Zedboardのコンソール上であらかじめ用意さてあるユーザ**ubuntu**としてログインします。  

```
# su ubuntu
$ cd
```

ubuntuでログインできたら、ROSの設定をします。

```
$ source /opt/ros/indigo/setup.bash
```

ROSのワークスペースを作ります。

```
$ mkdir -p catkin_ws/src
$ cd catkin_ws
```

ROSにおけるビルドは`catkin_make`というコマンドを使用します。一度、ビルドします。

```
$ catkin_make
```

また、ZedboardのIPアドレスを調べます。

```
$ ifconfig
eth0      Link encap:Ethernet  HWaddr 00:0a:35:00:01:22  
          inet addr:192.168.10.226  Bcast:192.168.10.255  Mask:255.255.255.0
          UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
          RX packets:2658 errors:0 dropped:0 overruns:0 frame:0
          TX packets:21 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:1000 
          RX bytes:303632 (303.6 KB)  TX bytes:2282 (2.2 KB)
          Interrupt:54 Base address:0xb000 

lo        Link encap:Local Loopback  
          inet addr:127.0.0.1  Mask:255.0.0.0
          UP LOOPBACK RUNNING  MTU:65536  Metric:1
          RX packets:8 errors:0 dropped:0 overruns:0 frame:0
          TX packets:8 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:0 
          RX bytes:592 (592.0 B)  TX bytes:592 (592.0 B)
```

<font color="red">PC上における作業</font>  
PCで生成したコンポーネントをZedboardに転送します。  
IP_ADDRESSは直前にZedboard上において調べたIPアドレスに置き換えてください。  
また、パスワードを聞かれます。パスワードは<font color="red">**ubuntu**</font>です。

```
$ cd ~/exp_dir/component_pwm_ctl/software/ros_package/
$ scp -r component_pwm_ctl ubuntu@IP_ADDRESS:/home/ubuntu/.
ubuntu@IP_ADDRESS's password: 
CMakeLists.txt                                100%  692     0.7KB/s   00:01
Component_pwm_ctlMsg.msg                      100%   47     0.1KB/s   00:00
package.xml                                   100%  890     0.9KB/s   00:00
bridge.py                                     100% 1905     1.9KB/s   00:00
component_pwm_ctl_node.py                     100% 1204     1.2KB/s   00:00
test_node.py                                  100%  671     0.7KB/s   00:00
```

<font color="red">Zedboard上における作業</font> 

PCから転送したディレクトリをROSのワークスペースにコピーします。

```
$ cd
$ cp -r component_pwm_ctl/ catkin_ws/src/.
```

コンポーネントの中身を確認します。

```
$ cd ~/catkin_ws/src/component_pwm_ctl
$ ls
CMakeLists.txt  include  msg  package.xml  scripts
```

コンポーネントはROSのパッケージ構造に準拠しています。

- CMakeLists.txt
	- ビルド設定
- include
	- ヘッダファイルなどを入れるディレクトリ
- msg
	- メッセージファイルを入れるディレクトリ
- package.xml
	- パッケージの情報が記述されてある
- scripts
	- プログラムを入れるディレクトリ

また、`scripts`下にはcReCompによって生成されたプログラムがあります。

- bridge.py
	- Xillybusのデバイスドライバにアクセスするためのラッパソフトウェア
- component_pwm_ctl_node.py
	- ROSのメッセージによって、FPGAへデータを入出力するためのソフトウェアインターフェイス
- test_node.py
	- 生成したROS準拠FPGAコンポーネントの動作確認をするためテストプログラム


テストプログラム`~/catkin_ws/src/component_pwm_ctl/scripts/test_node.py`を以下のように編集します。


```diff
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

+       msg.input_dir_in = 1
+       msg.input_para_in = 12000

        while not rospy.is_shutdown():
                pub.publish(msg)
                r.sleep()

def callback(data):
        # please desicribe your code
        pass

def component_pwm_ctl_sub(self):
        rospy.init_node('component_pwm_ctl_sub', anonymous=True)
        rospy.Subscriber('component_pwm_ctl_output', Component_pwm_cack)
        rospy.spin()

+if __name__ == '__main__':
+	component_pwm_ctl_pub()
```

プログラムに実行権限をつけます。

```
$ cd ~/catkin_ws/src/component_pwm_ctl/scripts
$ chmod +x component_pwm_ctl_node.py test_node.py
```

コンポーネントをビルドします。以下のようなビルドログが出力されます。

```
$ cd ~/catkin_ws/
$ catkin_make
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - component_pwm_ctl
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'component_pwm_ctl'
-- ==> add_subdirectory(component_pwm_ctl)
-- Using these message generators: gencpp;genlisp;genpy
-- component_pwm_ctl: 1 messages, 0 services
-- Configuring done
-- Generating done
-- Build files have been written to: /home/ubuntu/catkin_ws/build
####
#### Running command: "make -j2 -l2" in "/home/ubuntu/catkin_ws/build"
####
Scanning dependencies of target _component_pwm_ctl_generate_messages_check_deps_Component_pwm_ctlMsg
Scanning dependencies of target std_msgs_generate_messages_py
[  0%] Built target std_msgs_generate_messages_py
Scanning dependencies of target sensor_msgs_generate_messages_py
[  0%] Built target sensor_msgs_generate_messages_py
Scanning dependencies of target std_msgs_generate_messages_lisp
[  0%] Built target std_msgs_generate_messages_lisp
[  0%] Built target _component_pwm_ctl_generate_messages_check_deps_Component_pwm_ctlMsg
Scanning dependencies of target sensor_msgs_generate_messages_lisp
Scanning dependencies of target std_msgs_generate_messages_cpp
[  0%] Built target sensor_msgs_generate_messages_lisp
[  0%] Built target std_msgs_generate_messages_cpp
Scanning dependencies of target sensor_msgs_generate_messages_cpp
Scanning dependencies of target component_pwm_ctl_generate_messages_py
[  0%] Built target sensor_msgs_generate_messages_cpp
[ 25%] Generating Python from MSG component_pwm_ctl/Component_pwm_ctlMsg
Scanning dependencies of target component_pwm_ctl_generate_messages_lisp
[ 50%] Generating Lisp code from component_pwm_ctl/Component_pwm_ctlMsg.msg
[ 75%] Generating Python msg __init__.py for component_pwm_ctl
[ 75%] Built target component_pwm_ctl_generate_messages_lisp
Scanning dependencies of target component_pwm_ctl_generate_messages_cpp
[100%] Generating C++ code from component_pwm_ctl/Component_pwm_ctlMsg.msg
[100%] Built target component_pwm_ctl_generate_messages_py
[100%] Built target component_pwm_ctl_generate_messages_cpp
Scanning dependencies of target component_pwm_ctl_generate_messages
[100%] Built target component_pwm_ctl_generate_messages
```

ROSではソフトウェアを起動する際に、ネームサービスなどの役割を果たす**roscore**を起動します。  
以下のような起動ログが出たら、Enterキーを数回押します。

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roscore &

logging to /home/ubuntu/.ros/log/a014889c-c078-11e6-a86c-000a35000122/roslaunch-ubuntu-armhf-4720.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu-armhf:52630/
ros_comm version 1.11.20


SUMMARY
========

PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.20

NODES

auto-starting new master
process[master]: started with pid [4731]
ROS_MASTER_URI=http://ubuntu-armhf:11311/

setting /run_id to a014889c-c078-11e6-a86c-000a35000122
process[rosout-1]: started with pid [4744]
started core service [/rosout]

ubuntu@ubuntu-armhf:~/$
ubuntu@ubuntu-armhf:~/$
```

コンポーネントを起動します。

```
$ rosrun component_pwm_ctl component_pwm_ctl.py
```

ここで、ssh接続などでコンソールをもう1つ起動します。そのコンソール上で以下のコマンドでテストプログラムを起動します。

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun component_pwm_ctl test_node.py
```

`test_node.py`を起動して、モータが動き始めたら動作確認は成功です。