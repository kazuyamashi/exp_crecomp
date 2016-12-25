#readme

### 被験者募集

- 実験期間：12月12日〜16日、26日、27日、28日 <font color="red">**日にちは相談に乗ります。**</font>  
- 実験内容：コンポーネント自動生成ツールcReCompを用いたコンポーネント開発  
- 実験時間：10:00〜17:00 (7時間, 休憩含む)  
- 謝金	：950円×7時間分  
- 備考
	- 実験前に事前導入資料をお渡ししますので、あらかじめ、一読していただく必要があります。
	- 実験にご協力いただける場合、研究室ミーティングの後、日程調整を行います。

exp1@192.168.8.11
exp2@192.168.8.13
exp3@192.168.8.12
exp4@192.168.8.16

sudo apt-get install python-pip iverilog -y 
git clone https://github.com/kazuyamashi/cReComp.git

git clone https://github.com/PyHDI/pyverilog.git
cd pyverilog/
sudo python setup.py install

git clone https://github.com/PyHDI/veriloggen.git
cd veriloggen/
python setup.py install

sudo apt-get install python-jinja2