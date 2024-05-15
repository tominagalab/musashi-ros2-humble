
import sys
import socket

MAX_RECV_SIZE = 1024 # byte

class RefBoxClient:
  def __init__(self,):
    # ソケットの作成，ソケットはメンバ変数
    # socket.SOCK_DGRAMオプションでUDP通信の設定
    self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
  def connect(self, address, port):
    print('[{}] Conenct to {}:{}'.format(sys._getframe().f_code.co_name, address, port))
    send_len = self._socket.sendto('hello'.encode('utf-8'), (address, port))
    # print('send bytes: ', send_len)
    
    # blockingモードでタイムアウトを設定する
    self._socket.settimeout(2)
    try:
      recv, address = self._socket.recvfrom(MAX_RECV_SIZE) # ブロッキングモードで受信待ち
    except socket.timeout as e: # タイムアウトエラー
      print('Error: ', e)
      return False
    
    if address == REFBOX_ADDRESS:
      print(recv)
      # RefereeBoxに無事接続できていれば，
      # 返信として以下のようなWELCOMEメッセージが返ってくるはず
      # {
      #   "command": "WELCOME",
      #   "targetTeam": "224.16.32.*"
      # }
    else:
      return False
    
  def disconnect(self,):
    print('[{}] Disconnect'.format(sys._getframe().f_code.co_name))
    self.socket.close()