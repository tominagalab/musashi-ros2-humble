# hardware  
センサ，アクチュエータなど外部デバイス用パッケージディレクトリ  

## フォルダ構成  
- hardware（このフォルダ）  
  - musashi_camera（全方位カメラ用パッケージディレクトリ）  
  - musashi_motor（駆動モータ用パッケージディレクトリ）  

## 関連ライブラリ　　

### neoAPI  
Baumerカメラ用ライブラリ  
Baumer公式サイトからダウンロード（要Email）  

### EPOS library  
Maxon EPOS用ライブラリ  
1. 以下のコマンドにて好きなディレクトリにzipファイルをダウンロード  
`wget https://www.maxongroup.co.jp/medias/sys_master/root/8994700394526/EPOS-Linux-Library-En.zip`  
1. zipファイルを解凍する  
1. EPOS_Linux_Libraryフォルダ内のinstall.shをsudoで実行する  
1. ライブラリファイル（.hや.soなど）は/optディレクトリにコピーされているはず  
