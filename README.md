# musashi-ros2-humble
本パッケージは Robocup Hibikino-Musashi のROS2用リポジトリになります．
Visual Studioを用いたC++のみの開発に限界を感じてきたため，
レフェリーボックスの仕様変更に合わせて新たなUbuntuベースのHibikino-musashiとすべく，開発を開始する．    

## 開発環境  
- Ubuntu 22.04  
- ROS2 humble  
- Visual Studio Code（エディタ）  
- python，C++  

## コーディング規約（命名規則）  
pythonでのコーディングにおいてはPEP8コードスタイルに準拠する．
以下を参照すること．  
`https://peps.python.org/pep-0008/#code-lay-out`  
`https://atmarkit.itmedia.co.jp/ait/articles/2308/08/news020.html`  

## ディレクトリ構成     
<pre>
musashi-ros2-humble（ルートディレクトリ）  
├── README.md
├── behavior
├── coachbox
├── hardware
├── localization
├── musashi_msgs
└── perception 
</pre>

|Name|Detail|  
|---|---|
|README.md|このファイル|
|behavior|ルールベース行動決定，ステートマシン，行動選択のディレクトリ|
|coachbox|レフェリーボックス，プレイヤーとの通信関係ディレクトリ|
|hardware|外部デバイス制御用のディレクトリ|
|localization|自己位置推定用のディレクトリ|
|musashi_msgs|独自メッセージ用のディレクトリ|
|perception|外界認識，知覚系のディレクトリ|

## パッケージ作成コマンド例  
- pythonパッケージ  
``ros2 pkg create [package name] --build-type ament_python --dependencies rclpy``  
- C++パッケージ  
``ros2 pkg create [package name] --build-type ament_cmake --dependencies rclcpp``  
- pythonのrqtプラグインパッケージ  
`ros2 pkg create [package name] --build-type ament_python --dependencies rclpy python_qt_binding rqt_gui rqt_gui_py rqt_py_common`