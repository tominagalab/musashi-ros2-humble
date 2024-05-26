from python_qt_binding.QtCore import QThread, Signal

import sys
import socket
import threading
import time
import json


TEAM_IP = '172.16.32.44' # チームに割り振られた固有IP（コーチボックスに設定するアドレスではありません）
MAX_RECV_SIZE = 1024*4 # byte

class RefBoxClient(QThread):
  
  # シグナル定義
  recievedCommand = Signal(str,str)
  
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
      self._socket.settimeout(3) # timeout [sec]
      self._socket.connect((address, port))
    except Exception as e:
      print(e)
      return False
    
    # 無事にサーバ（RefereeBox）へ接続ができれば，RefereeBox側で反応がある
    return True # 接続完了を表す
    
  # 切断処理メソッド
  def disconnect(self,):
    self._isRun = False
    self._socket.close()
    self.join()
    
  # 主となる処理（スレッド処理実態）
  def run(self,):
    # ブロッキングモード設定
    self._socket.settimeout(None)

    while self._isRun:
      # RefereeBoxからのコマンド受信待ち
      recv = self._socket.recv(MAX_RECV_SIZE) 
      
      # コマンドは全てjson形式のテキストデータで送られてきます
      # データ末尾にはNULL（'\0'）が入れられているみたいです
      # 文字列（str）型の末尾１文字を削除している
      recv_json = json.loads(recv.decode('utf-8')[:-1])
      
      # recv_jsonは辞書型
      # キーは'command'と'targetTeam'の二つ，各値を取得する
      command = recv_json['command']
      targetTeam = recv_json['targetTeam']
      
      # シグナルの発行      
      self.recievedCommand.emit(command, targetTeam)