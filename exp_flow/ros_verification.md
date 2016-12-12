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

```
root@ubuntu-armhf:~# 
```
