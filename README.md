# musashi-ros2-humble
Robocup Hibikino-Musashi のROS2用パッケージ 

## 開発環境  
- Ubuntu 22.04  
- ROS2 humble  

## ディレクトリ構成  
- musashi-ros2-humble  
  - README.md（このファイル）
  - hardware（センサ，アクチュエータなど外部デバイス用メタディレクトリ）  
  - perception（画像処理や物体認識など知覚系メタディレクトリ）  
  - localization（自己位置推定用メタディレクトリ） 
  - behavior（行動決定，意思決定用メタディレクトリ） 
  - coachbox（CoachBox用メタディレクトリ）  
  - musashi_msgs（ROSメッセージ開発用パッケージディレクトリ）

## パッケージ作成コマンド例  
### pythonパッケージ　　
``ros2 pkg create ament_python ``  
### C++パッケージ　　
``ros2 pkg create [package name] --build-type ament_cmake --dependencies rclcpp``  