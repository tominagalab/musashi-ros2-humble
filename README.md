# musashi-ros2-humble
本パッケージは Robocup Hibikino-Musashi のROS2用リポジトリになります．
Visual Studioを用いたC++のみの開発に限界を感じてきたため，
レフェリーボックスの仕様変更に合わせて新たなUbuntuベースのHibikino-musashiとすべく，開発を開始する．    

## 開発環境  
- Ubuntu 22.04  
- ROS2 humble  
- Visual Studio Code（エディタ）  
- python，C++  

## 開発規約  
PEP8コードスタイルに準拠する．  
`https://peps.python.org/pep-0008/#code-lay-out`  

## ディレクトリ構成   
<pre>
musashi-ros2-humble（ルートディレクトリ）  
|-- README.md（このファイル）  
|-- behavior（ルールベース行動選択，意思決定パッケージ用ディレクトリ）  
|-- coachbox（コーチボックス関連パッケージ用ディレクトリ）  
|-- hardware（外部デバイス関連パッケージ用ディレクトリ）    
|-- localization（自己位置推定関連パッケージ用ディレクトリ）  
|-- musashi_msgs（独自メッセージ用ディレクトリ）  
`-- perception（外界認識等，知覚系パッケージ用ディレクトリ）  
</pre>

## パッケージ作成コマンド例  
### pythonパッケージ　　 
``ros2 pkg create [package name] --build-type ament_python --dependencies rclpy``  
### C++パッケージ　　
``ros2 pkg create [package name] --build-type ament_cmake --dependencies rclcpp``  