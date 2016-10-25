#ROS on xillinux on zybo

#### 1. Xillinuxの導入
xillinuxのドキュメント参照

#### 2. xillinuxのディストリビューションアップグレード
以下の作業はrootユーザでやる。
アップグレードコマンドは以下のもの。途中で質問されるが基本的にYes

```
# do-release-upgrade
```

おそらくインストール中は色々こける(正常にインストールされない)。以下のコマンドで失敗したものは再インストールできる。
（以下のコマンドをひたすら繰り返す）

```
# apt-get upgrade
# apt-get -f install
```

ある程度エラーがなくなれば終了。ここからはユーザを作って作業する。

```
# adduser your_name
# gpasswd -a user_name sudo
$ su your_name
$ cd
```

この時点でエディタやgitを入れておく。滞りなくinstallできたらOK

```
$ sudo apt-get install emacs git -y
```

#### 4. pipのインストール

```
$ wget https://bootstrap.pypa.io/get-pip.py
$ sudo python get-pip.py
```


#### 3. ROS indigoのインストール
リポジトリの設定

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
```

ROS Indigoをソースインストールする。最初に依存関係解決のためのツールをインストール。

```
$ sudo apt-get install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential
```

もし、apt-getで入らなければpipで入れる。（build-essentialはaptでしか入らない）

```
$ sudo pip install rosdep rosinstall-generator wstool rosinstall
```

rosdepでROSをインストールするための準備をする。具体的にはROSのリポジトリの更新。この時`rosdep update`は**sudo**をつけないことに注意

```
$ sudo rosdep init
$ rospdep update
```

install用のディレクトリを作る。

```
$ mkdir $HOME/ros_catkin_ws
$ cd $HOME/ros_catkin_ws
```

rosinstall_generatorはROSのパッケージをインストールする際のインストーラを生成してくれる。生成したインストーラはwstoolで実行する。  
以下のコマンドでROS本体のインストール（ソースの取得）を始める

```
$ rosinstall_generator ros_comm --rosdistro indigo --deps --wet-only --tar > indigo-ros_comm-wet.rosinstall
$ wstool init -j8 src indigo-ros_comm-wet.rosinstall
```

ROSのビルドを始める前に依存関係の解消をする。

```
$ rosdep install --from-paths src --ignore-src --rosdistro indigo -y
```

この際、以下のようなエラーが出る場合がある。

```
E: Package 'sbcl' has no installation candidate
ERROR: the following rosdeps failed to install
  apt: command [sudo apt-get install -y sbcl] failed
```

sbclはroslispで使用するが、今回は必要ではないのでインストールエントリから削除する。また、src内のディレクトリも削除する。

```
$ cd src
$ wstool rm roslisp
$ rm -rf roslisp
$ cd $HOME/ros_catkin_ws
```

バージョンは1.54、1.64、1.84があったが1.84をインストール

```
$ sudo apt-get install libboost1.54-dev
$ sudo apt-get install libboost-dev
$ sudo apt-get install libboost-graph-dev
```

```
$ sudo apt-get install libboost-python1.48-dev
```

gccxmlがないと言われる

```
$ wget http://launchpadlibrarian.net/153397037/gccxml_0.9.0+git20130511-1ubuntu2_armhf.deb
$ dpkg -i gccxml_0.9.0+git20130511-1ubuntu2_armhf.deb
```

```
$ sudo apt-get install libboost1.48-all-dev
```

####参考サイト

- http://wiki.ros.org/Installation/UbuntuARM (indigo)
- http://wiki.ros.org/indigo/Installation/Source
- http://forestofazumino.web.fc2.com/ros/ros_install.html


libboost-dev
libboost-chrono-dev
libboost-date-time-dev
libboost-filesystem-dev
libboost-graph-dev
libboost-graph-parallel-dev
libboost-iostreams-dev
libboost-locale-dev
libboost-math-dev
libboost-mpi-dev
libboost-mpi-python-dev
libboost-program-options-dev
libboost-python-dev
libboost-random-dev
libboost-regex-dev
libboost-serialization-dev
libboost-signals-dev
libboost-system-dev
libboost-test-dev
libboost-thread-dev
libboost-timer-dev
libboost-wave-dev

```
# See http://help.ubuntu.com/community/UpgradeNotes for how to upgrade to
# newer versions of the distribution.

deb http://ports.ubuntu.com/ubuntu-ports/ trusty main restricted
deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty main restricted

## Major bug fix updates produced after the final release of the
## distribution.
deb http://ports.ubuntu.com/ubuntu-ports/ trusty-updates main restricted
deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty-updates main restricted

## Uncomment the following two lines to add software from the 'universe'
## repository.
## N.B. software from this repository is ENTIRELY UNSUPPORTED by the Ubuntu
## team. Also, please note that software in universe WILL NOT receive any
## review or updates from the Ubuntu security team.
# deb http://ports.ubuntu.com/ubuntu-ports/ precise universe
deb-src http://ports.ubuntu.com/ubuntu-ports/ precise universe
deb http://ports.ubuntu.com/ubuntu-ports/ precise-updates universe
deb-src http://ports.ubuntu.com/ubuntu-ports/ precise-updates universe

## N.B. software from this repository may not have been tested as
## extensively as that contained in the main release, although it includes
## newer versions of some applications which may provide useful features.
## Also, please note that software in backports WILL NOT receive any review
## or updates from the Ubuntu security team.
deb http://ports.ubuntu.com/ubuntu-ports/ precise-backports main restricted
deb-src http://ports.ubuntu.com/ubuntu-ports/ precise-backports main restricted

deb http://ports.ubuntu.com/ubuntu-ports/ trusty-security main restricted
deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty-security main restricted
deb http://ports.ubuntu.com/ubuntu-ports/ precise-security universe
deb-src http://ports.ubuntu.com/ubuntu-ports/ precise-security universe
deb http://ports.ubuntu.com/ubuntu-ports/ precise-security multiverse
deb-src http://ports.ubuntu.com/ubuntu-ports/ precise-security multiverse
```

https://launchpad.net/ubuntu/+source/boost1.46/1.46.1-7ubuntu3/+build/3060825