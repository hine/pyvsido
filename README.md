# PythonVSido_Library

## これは何？
[アスラテック株式会社](http://www.asratec.co.jp/ "アスラテック株式会社")のロボット制御マイコンボード「[V-Sido CONNECT RC](http://www.asratec.co.jp/product/connect/rc/ "V-Sido CONNECT RC")」をコントロールするためのPythonのライブラリです。  
[V-Sido Developerサイトの技術資料](https://v-sido-developer.com/learning/connect/connect-rc/ "V-Sido Developerサイトの技術資料")に公開されている情報を元に個人が作成したもので、アスラテック社公式のライブラリではありません。  

## 誰が作ったの？
アスラテック株式会社に勤務する今井大介(Daisuke IMAI)が個人として作成しました。

## 使い方
$ git clone git@github.com:hine/PythonVSido_Library.git  
$ cd PythonVSido_Library  
$ pip install .  
で、ライブラリを導入してください。  

ライブラリの使い方の例  
\>\>\> import vsido  
\>\>\> vc = vsido.Connect()  
\>\>\> vc.connect("COM3")  
\>\>\> vc.walk(100, 0)  

ライブラリの詳細は、docs/vsido.connect.htmlを見てください。  

## 免責事項
一応。  

このサンプルコードを利用して発生したいかなる損害についても、アスラテック株式会社ならびに今井大介は責任を負いません。自己責任での利用をお願いします。  

## ライセンス
このライブラリは、MITライセンスで配布します。MITライセンスについてはLICENSE.txtを見てください。  
This software is released under the MIT License, see LICENSE.txt.
