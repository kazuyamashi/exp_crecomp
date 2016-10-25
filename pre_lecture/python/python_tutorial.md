
#　Python文法チュートリアル　(Kazushi Yamashina)

- このチュートリアルではプログラミング言語**Python**の基本的な文法とプログラムの実行方法を紹介します．
- Pythonのご経験がある方はこのチュートリアルは必要ありませんがお持ちのPCにPythonを実行できる環境構築をお願い致します．

<a name="contents"></a>
## Contents
各項目からこの一覧に戻れるようになっています．

<!-- MarkdownTOC autolink="true" autoanchor="true" bracket="round" depth="6"-->

- [Pythonを始める準備](#pythonを始める準備)
- [Pythonのプログラムを実行する](#pythonのプログラムを実行する)
- [Pythonの文法](#pythonの文法)
	- [インデントとコメントアウト](#インデントとコメントアウト)
	- [変数宣言と標準出力](#変数宣言と標準出力)
	- [数値演算](#数値演算)
	- [条件分岐文（if, elif, else）](#条件分岐文（if-elif-else）)
	- [繰り返し文](#繰り返し文)
	- [リスト](#リスト)
	- [関数](#関数)
	- [クラス](#クラス)
- [練習課題](#練習課題)
- [Reference](#reference)
- [お問合せ・質問](#お問合せ・質問)

<!-- /MarkdownTOC -->


<a name="pythonを始める準備"></a>
## Pythonを始める準備
下記に示すのは一般的なPythonの実行環境の一覧です．  

|              Python実行環境              |  動作OS |                              インストール方法(参考Webサイト)                             |
|------------------------------------------|---------|------------------------------------------------------------------------------------------|
| [Python](https://www.python.org/) (おすすめ) | Windows | [windows環境へのpythonインストール](http://qiita.com/maisuto/items/404e5803372a44419d60)|
|                                          | OS X    | MacのOSではPythonがプリインストールされています（MacのTerminalで実行する）               |
|                                          | Ubuntu  | sudo apt-get isntall python                                                              |
| [Cygwin](https://www.cygwin.com/)        | Windows | [Cygwinのインストール方法&設定方法](http://musashi.osdn.jp/cygwin/cygwin.html)           |

[Contentsに戻る](#contents)

<a name="pythonのプログラムを実行する"></a>
## Pythonのプログラムを実行する
Pythonのプログラムを記述するためのファイル拡張子は`.py`です．テキストファイルを作成し，拡張子を`.py`に変更してください．

```python
#hello.py
print "hello world"
```

例えば上に示すような*hello world*と出力するための簡単なプログラムhello.pyを実行してみましょう．

```
python hello.py
hello world
```
出力結果には**hello world**と出力されました．Pythonではこのように`python プログラム名`というように実行します．  

[Contentsに戻る](#contents)

<a name="pythonの文法"></a>
## Pythonの文法
Pythonの基本的な文法を紹介します．また、コード例中の`# ==== result ====`は実行した際の出力結果です．  
`# ==== result ====`がある例では、そのまま実行していただければ基本的には実行できるかと思います．  

<a name="インデントとコメントアウト"></a>
### インデントとコメントアウト
Pythonにおけるインデントはほかの言語と比べ，比較的重要な役目があり，C言語やJavaなどの**{ }**と同じ働きをします．つまり，ブロック構造をインデントで表現します．これがPythonにおける特長の１つです．
例えばC言語で以下のようなif文の構造があるとします．

```c
// C
if (A < B){
	printf("A is smaller than B\n");
	B = B + 1;
}
else if ( A > B ){
	printf("A is larger than B\n");
	A = A + 1;
}
else{
	printf("A and B are equal \n");
}

```
これをPythonにおいて表現すると，以下のようになります．

```python
# Python
if A < B:
	print "A is smaller than B"
	B = B + 1
elif A > B:
	print "A is larger than B"
	A = A + 1
else:
	print "A and B are equal"
```
このようにif文でしたら，条件文の下に同じレベルのインデントで処理を書くとブロック構造ができます．
この際，スペースとタブの混在にすると実行エラーとなってしまいますので注意してください．  
またPythonでは文頭に`#`をつけるとコメントになります．

[Contentsに戻る](#contents)

<a name="変数宣言と標準出力"></a>
### 変数宣言と標準出力
Pythonでは明確な変数宣言は必要ありません．なお，リストなどの機能を使うときは，初期化として宣言する場合もあります（後で説明します）．  
それゆえに，変数に任意の値（文字含む）を代入するとインタプリタが自動的に型を認識してくれます．

```python
string = "Hello World"
num_a = 100
num_b = 10.21
```

またPythonにおける標準出力関数はprint文を用います．printという記述のあと，ダブルコーテーションで囲むとそれが文字列として出力されます．  
また，先ほどの変数を指定しても，各要素の中身が表示可能です．

```python
print "Hello World"
# ==== result ====
# Hello World

string = "Hello World"
num_a = 100
num_b = 10.21

print string
print num_a
print num_b
# ==== result ====
# Hello World
# 100
# 10.21
```

また，C言語のprintf関数と同じようにフォーマットを指定して，英数字と文字列を交えて出力することもできます．

```python
name = "Taro"
years = 23

print "%s is %d years old"%(name, years)
# ==== result ====
# Taro is 23 years old
```

また，文字列として宣言した*数字*を数値へ変換することも可能であり，その逆も可能です．

```python
number = 100
string = "100"

# string => integer number
int(string)

# integer number => string
str(number)

```

[Contentsに戻る](#contents)

<a name="数値演算"></a>
### 数値演算
Pythonでもほかの言語と同様，数値演算をすることが可能です．下記の例では`# -*- coding: utf-8 -*-`という記述がありますが，こちらはソースコード中の文字のエンコードを指定する記述です．特に，ソースコード中に日本語を含む場合は書きます．

```python
# -*- coding: utf-8 -*-
x = 20
y = 5

print x + y # 加算
print x - y # 減算
print x * y # 乗算
print x / y # 除算

# ==== result ====
# 25
# 15
# 100
# 4
```

[Contentsに戻る](#contents)

<a name="条件分岐文（if-elif-else）"></a>
### 条件分岐文（if, elif, else）
Pythonにおける条件分岐文は`if`，`elif`，`else`を用いて構成します．この際，条件文の後にコロン`:`をつけるのを忘れないでください．

```python
A = 10
B = 20

if A < B:
	print "A is smaller than B"
elif A > B:
	print "A is larger than B"
else:
	print "A and B are equal"

# ==== result ====
# A is smaller than B
```

なお、Pythonにおける論理演算子が少し特殊です．例えばC言語において以下のような条件分岐構造があった場合、

```c
if ( A < B && A < 0 ){
	//processing
}
else if( A == 0 || B == 0 ){
	//processing
}
```
Pythonで記述すると以下のようになります．

```python
if A < B and A < 0:
	# processing
elif A == 0 or B == 0:
	# processing
```

[Contentsに戻る](#contents)

<a name="繰り返し文"></a>
### 繰り返し文
Pythonにおける繰り返し文は，for文とwhile文を使います．  
for文では**range関数**を使うのが便利です．

```python
# -*- coding: utf-8 -*-

# range(start_index, cycle)
# : 第1引数にはループインデックスの開始番号、第2引数には繰り返し回数を指定
# 以下のように記述することによって、インデックス変数となる．
for i in range(0, 5):
	print "i = ", i
# ==== result ====
# i =  0
# i =  1
# i =  2
# i =  3
# i =  4

i = 0
while i < 5:
	print "i = ", i
	i = i + 1
# ==== result ====
# i =  0
# i =  1
# i =  2
# i =  3
# i =  4

```


[Contentsに戻る](#contents)

<a name="リスト"></a>
### リスト
リストを使用する場合、「最初に初期化する値はないけどリストとして使いたい」という場合、変数の後に`= []`という風に記述すると任意のリスト宣言となります．リストは、C言語でいう**配列**と似たものです．リストに任意の要素を追加したい場合、**append関数**を使用します．

```python
# -*- coding: utf-8 -*-

list_x = []
# 5回のループでそれぞれiの値をlist_xに追加しています．
for i in range(0, 5):
	# append() : 配列へ追加したい要素を引数として指定する
	list_x.append(i)

# 以下のように記述すると、list_xの中にあるすべての要素に対して処理を行うfor文となります．
# これはin演算子の特徴であり、inの後にある対象オブジェクトの1要素がelemという変数に代入されていきます．
for elem in list_x:
	print elem
# ==== result ====
# 0
# 1
# 2
# 3
# 4
```

もちろん、最初の初期化も可能です．またappend関数以外にもリストで使用する関数を紹介します．

```python
# -*- coding: utf-8 -*-

list_x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

# insert() : インデックスを指定して挿入
list_x.insert(2,10)
print "list_x.insert(2,10) : ", list_x

# remove() : リスト中に指定した引数の値があったら削除
list_x.remove(3)
print "list_x.remove(3) : ", list_x

# pop() : リストの末尾を削除．引数を指定すると、そのインデックスの要素が削除される．
list_x.pop()
print "list_x.pop() : ", list_x

# index() : 引数で指定した値のインデックスを取得
print "list_x.index(10) : ", list_x.index(10)

# インデックスを指定しての値の変更
list_x[1] = 100
print "list_x[1] = 100 : ", list_x

# len() リストのd長さを取得
print "list_x length : ", len(list_x)

# ==== result ====
# list_x.insert(2,10) :  [0, 1, 10, 2, 3, 4, 5, 6, 7, 8, 9]
# list_x.remove(3) :  [0, 1, 10, 2, 4, 5, 6, 7, 8, 9]
# list_x.pop() :  [0, 1, 10, 2, 4, 5, 6, 7, 8]
# list_x.index(10) :  2
# list_x[1] = 100 :  [0, 100, 10, 2, 4, 5, 6, 7, 8]
# list_x length :  9
```

[Contentsに戻る](#contents)
<a name="関数"></a>
### 関数
Pythonにおいて関数宣言をする際は`def`を使用します．defの後に関数名、また`( )`の中には引数を指定します．  
`( )`の後にコロン`:`をつけるのを忘れないでください．

```python
# -*- coding: utf-8 -*-

# 引数なしの関数
def print_hello():
	print "hello"

# 引数ありの関数
# 戻り値はreturnで返す
def add( A, B ):
	return A + B

# 関数に与える引数は以下のようにすることで初期化できる
def print_info(name = "no name", age = 0, height = 100, weight = 50):
	print "name: %s age: %d height: %d weight: %d"%(name, age, height, weight)

#　各関数の実行
print_hello()

print add(100,300)

# 引数なし
print_info()
# 引数あり
print_info("taro", 20, 170, 60)

# ==== result ====
# hello
# 400
# name: no name age: 0 height: 100 weight: 50
# name: taro age: 20 height: 170 weight: 60
```

[Contentsに戻る](#contents)
<a name="クラス"></a>
### クラス
Pythonのクラス宣言は関数宣言と似ています．書式としては`class クラス名(継承するクラス"省略可")：`という風に宣言します．  
以下に実際の例を示します．
`__init__`はPythonにおいてクラスを定義する際、多くの場合定義する関数です．  
この関数ではクラスの各メンバ変数の初期化を行います．つまり，コンストラクタのような働きをします．  
また、
それぞれのメンバ関数、`__init__`の引数にある**self**はクラス自身のインスタンスを表しているため各メンバ関数に必ず必要な引数です．  

```python
# -*- coding: utf-8 -*-

class Person:
	def __init__(self, name = None, age = 0, height = 100, weight = 50):
		# メンバ変数の初期化
		# メンバ変数にする場合は必ずselfをつける
		# selfがついている変数スコープはクラス内すべてにわたる
		# selfをつけない場合の変数スコープはその関数内に留まる
		self.name = name
		self.age = age
		self.height = height
		self.weight = weight
		self.classname = None
		self.bmi = 0

	# メンバ関数例
	def print_info(self):
		print "name: %s age: %d height: %d weight: %d"%(self.name, self.age, self.height, self.weight)

	def set_class(self, classname):
		# メンバ変数にアクセスする場合
		self.classname = classname
		return self.classname

	def calc_bmi(self):
		height = self.height/100.0
		weight = self.weight
		self.bmi = weight / (height*height)
		return self.bmi

# クラスのオブジェクトを作成
person = Person("taro", 20, 170, 60)

# 各メンバ関数の実行
person.print_info()
print person.set_class("1-B")
print person.calc_bmi()

```


[Contentsに戻る](#contents)

<a name="練習課題"></a>
## 練習課題
基本的に[このサイト](http://www.python-izm.com/)をみればPythonのことについてはわかります。

1. 1から50までの数を足して出力
- バブルソートをしてみる
	- 入力数字列は`input_array = [98,32,1,66,23,44,232,56,6,77]`とする
- フィボナッチ数列を関数にしてみる
	- 仕様
		- **引数**を与えると**返り値**としてそのインデックスの値が帰ってくる
		- 例 ` print fibo(1)` 結果 `1`
- **ファイル入出力を**してみる
	- 仕様
		- input.txtというファイルを用意して中身に任意の文字列を書いておく
		- 中身をoutput.txtというファイルで出力
- **コマンドライン引数**を取ってそれに応じた出力をする
	- 仕様
		- プログラムの実行時に`python sample.py 2`の2をコマンドライン引数という
		- 与えられた数字に応じて決まった文字列を出力する。(3パターンほど)
- **クラス**を作る
	- `ckass.py`を参考に下記の機能を満たす**メソッド**を実装する
		- 名前をセット/取得する
		- 数学の点数をセット/取得する
		- 英語の点数をセット/取得する
		- 平均を計算して取得する
		- 全ての情報を出力する
- 辞書を作ってみる（英語のみ）
	- 仕様
		- 登録モード`python dictionary.py register 単語　意味`
		- 検索モード`python dictionary.py search 単語`
		- 削除モード`python dictionary.py 単語`
		- 辞書データはどのようにしても可能
			- 例えば：アルファベット順にテキストファイルを分けてそこに保存

[Contentsに戻る](#contents)

<a name="reference"></a>
## Reference
- [Python入門から応用までの学習サイト](http://www.python-izm.com/)

[Contentsに戻る](#contents)

<a name="お問合せ・質問"></a>
## お問合せ・質問

kazushi@virgo.is.utsunomiya-u.ac.jp


[Contentsに戻る](#contents)
