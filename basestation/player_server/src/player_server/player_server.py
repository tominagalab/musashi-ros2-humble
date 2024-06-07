from python_qt_binding.QtCore import QThread, Signal

import sys
import socket
import threading

IP='0.0.0.0'
PORT=12536
MAX_RECV_SIZE=4096

class PlayerServer(QThread):
  
  # �V�O�i����`
  recievedPlayerData = Signal(int,dict)
  
  # �R���X�g���N�^
  def __init__(self):
    super(PlayerServer, self).__init__()
    
    # �\�P�b�g�̍쐬�D�\�P�b�g�̓����o�ϐ�
    # IPv4, UDP�ݒ�
    self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    
    # �v���C���[�f�[�^���X�g
    self._players = [{},{},{},{},{}]
    
    self._isRun = True
  
   # �f�X�g���N�^
  def __del__(self,):
    pass
  
  def open(self,):
    # �o�C���h�i�R�t���j
    self._socket.bind((IP, PORT))
    
  def close(self,):
    self._isRun = False
  
  def run(self,):
    # �v���C���[����̎�M�X���b�h
    self._socket.settimeout(None)
    
    while self._isRun:
      # ��M�����i�u���b�L���O���[�h�j
      recv, addr = self._socket.recvfrom(MAX_RECV_SIZE)    
      # recv�ɂ͎�M�����f�[�^�i������j
      # addr�ɂ͑��M���̃A�h���X�i������j�������Ă���

      # addr���瑗�M���̃v���C���[��ID������o��
      # IP�A�h���X�̉�1���ł킩��
      player_no = addr.split('.')[-1]

      # recv�̕�������J���}�ŃX�v���b�g����
      recv_str = recv.decode() # bytes�I�u�W�F�N�g����str�I�u�W�F�N�g�֕ϊ�
      values = recv_str.split(',') #�J���}�ŕ�������->������I�u�W�F�N�g�̃��X�g�I�u�W�F�N�g�ɂȂ�
      
      print('recv(srt)=', recv_str)
      print('splited=', values)
      
      # ���X�g���玫���ɕϊ����Ă���
      # ���łɐ����^�ɕϊ����Ă���
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
      
      # �v���C���[�f�[�^���X�g�Ɋi�[
      self._players[player_no] = data
      
      # �V�O�i�����s
      self.recievedPlayerData.emit(player_no, data)