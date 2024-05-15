
import sys
import socket

REFBOX_ADDRESS = '172.16.1.2'
REFBOX_PORT = 12345

class RefBoxClient:
  def __init__(self,):
    # ソケットの作成，ソケットはメンバ変数
    # socket.SOCK_DGRAMオプションでUDP通信の設定
    self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
  def connect(self,):
    print('[{}]Conenct to {}:{}'.format(sys._getframe().f_code.co_name, REFBOX_ADDRESS, REFBOX_PORT))
    
  
  def disconnect(self,):
    self.socket.close()