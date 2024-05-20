# musashi_coachbox  
コーチボックス用のROS2パッケージ.  
rqtのプラグインとして開発します．  
## 参考資料  
RQtのプラグインとして開発する必要があるので少々開発の仕方がややこしい．  
こちらを参考に → https://zenn.dev/shotaak/articles/f698974b59a02d  
参考になるgithubリポジトリは以下．  
- rqt_topicリポジトリ： https://github.com/ros-visualization/rqt_topic/tree/humble  
pythonパッケージとして作成されています．  
- rqt_consoleリポジトリ： https://github.com/ros-visualization/rqt_console/tree/dashing-devel  
CMakeパッケージとして作成されています．  
## 開発環境  
### Qt Designer  
rqtプラグインのGUI画面を作るための使いやすいエディタです．  
#### インストール方法  
以下のコマンドで必要なパッケージをインストールするだけ．   
QtCreatorにQtDesignerが含まれているだけなので，QtCreatorは使わなくていい．  
```sudo apt install qtbase5-dev clang qtcreator```  
## 開発方法  
以下のファイルを編集し開発していきます．  
1. `resource/musashi_coachbox.ui`  
1. `src/musashi_coachbox/musashi_coachbox_widget.py`  
1. `src/musashi_coachbox/musashi_coachbox.py`  
### musashi_coachbox.ui  
テキストボックスやらボタンやら，UIの定義ファイルで中身はXMLで記述されています．  
resourceディレクトリに置いています.  
開発・編集はQt Designerで.uiフィアルを開いて行います．  
XML言語を勉強する必要はありません．
### musashi_coachbox_widget.py  
QWidgetクラスを継承したカスタムWidgetクラスのモジュールです．
src/musashi_coachboxディレクトリに置いています．  
カスタムWidgetクラスが一つだけ定義されており，その中で.uiファイルをロードして画面を描画しています．
### musashi_coachbox.py  
Pluginクラスを継承したカスタムPluginクラスのモジュールです．  
src/musashi_coachboxディレクトリに置いています.  
カスタムWidgetクラスが定義されたモジュールをimportしており，Pluginとして起動する機能を所有しています．
rqtはおそらくこいつを1番最初に読み込んで実行しているはず．
### その他機能毎に開発したモジュール  
#### refbox_client.py  
RefereeBoxとの通信を担当するRefBoxClientクラスが実装されたモジュール．  
musashi_coachbox.pyから使用します．  
## RefereeBoxとの通信について  
RefereeBoxとはTCP通信でコマンドのやり取りが行われる．(UDPではないので間違えないように！)  
また，通信はjson形式のテキストデータで行われる．  
### 接続シーケンス  
（シーケンス図をそのうち作成します．）
### その他  
## package.xml  
パッケージの依存関係を設定しているxmlファイル．
## plugin.xml  
プラグインとしての振る舞いを設定しているxmlファイル．  
