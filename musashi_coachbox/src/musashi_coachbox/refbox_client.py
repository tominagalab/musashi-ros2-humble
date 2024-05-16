
import sys
import socket
import threading
import time
import json

MAX_RECV_SIZE = 1024*4 # byte

class RefBoxClient(threading.Thread):
  # コンストラクタ
  def __init__(self,):
    super(RefBoxClient, self).__init__()
    
    # ソケットの作成．ソケットはメンバ変数
    # IPv4, TCP設定
    self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    self._isRun = True
  
  # デストラクタ
  def __del__(self,):
    pass
  
  # 接続（サーバへのログイン）処理メソッド
  def connect(self, address, port):
    # 接続処理
    try:
      print('Try to connect to RefereeBox')
      self._socket.settimeout(3) # timeout [sec]
      self._socket.connect((address, port))
    except Exception as e:
      print(e)
      return False
    
    # 無事にサーバ（RefereeBox）へ接続ができれば，RefereeBox側で反応がある
        
    return True # 接続完了を表す
    
  # 切断処理メソッド
  def disconnect(self,):
    print('Disconnect from RefereeBox')
    self._isRun = False
    self._socket.close()
    self.join()
    
  # 主となる処理（スレッド処理実態）
  def run(self,):
    while self._isRun:
      # RefereeBoxからのコマンド受信処理
      recv = self._socket.recv(MAX_RECV_SIZE) 
      
      # コマンドは全てjson形式のテキストデータで送られてきます
      # データ末尾にはNULL（'\0'）が入れられているみたいです
      
      recv_json = json.load(recv.decode('utf-8'))
      print(recv_json)
      
      time.sleep(0.1) # sleepしないとCPUリソースを大量に要求するので，適当な周期で回しています．