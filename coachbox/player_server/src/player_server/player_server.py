from python_qt_binding.QtCore import QThread, Signal

import sys
import socket
import threading

IP='0.0.0.0'
PORT=12536
MAX_RECV_SIZE=4096

class PlayerServer(QThread):
  
  # シグナル定義
  recievedPlayerData = Signal(int,dict)
  
  # コンストラクタ
  def __init__(self):
    super(PlayerServer, self).__init__()
    
    # ソケットの作成．ソケットはメンバ変数
    # IPv4, UDP設定
    self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    
    # プレイヤーデータリスト
    self._players = [{},{},{},{},{}]
    
    self._isRun = True
  
   # デストラクタ
  def __del__(self,):
    pass
  
  def open(self,):
    # バインド（紐付け）
    self._socket.bind((IP, PORT))
    
  def close(self,):
    self._isRun = False
  
  def run(self,):
    # プレイヤーからの受信スレッド
    self._socket.settimeout(None)
    
    while self._isRun:
      # 受信処理（ブロッキングモード）
      recv, addr = self._socket.recvfrom(MAX_RECV_SIZE)    
      # recvには受信したデータ（文字列）
      # addrには送信元のアドレス（文字列）が入っている

      # addrから送信元のプレイヤーのIDを割り出す
      # IPアドレスの下1桁でわかる
      player_no = addr.split('.')[-1]

      # recvの文字列をカンマでスプリットする
      recv_str = recv.decode() # bytesオブジェクトからstrオブジェクトへ変換
      values = recv_str.split(',') #カンマで分割処理->文字列オブジェクトのリストオブジェクトになる
      
      print('recv(srt)=', recv_str)
      print('splited=', values)
      
      # リストから辞書に変換しておく
      # ついでに整数型に変換しておく
      data = {
        'color': int(values[0]),
        'id': int(values[1]),
        'action': int(values[2]),
        'state': int(values[3]),
        'ball': {
          'distance': int(values[4]),
          'angle': int(values[5]),
        },
        'goal': {
          'distance': int(values[6]),
          'angle': int(values[7]),
        },
        'myGoal': {
          'distance': int(values[8]),
          'angle': int(values[9]),
        },
        'position': {
          'x': int(values[10]),
          'y': int(values[11]),
          'angle': int(values[12])
        },
        'role': int(values[13]),
        'haveBall': int(values[14]),
        'move_to': {
          'x': int(values[15]),
          'y': int(values[16]),
          'angle': int(values[17])
        },
        'obstacle': {
          'distance': int(values[18]),
          'angle': int(values[19])
        }
      }
      
      # プレイヤーデータリストに格納
      self._players[player_no] = data
      
      # シグナル発行
      self.recievedPlayerData.emit(player_no, data)