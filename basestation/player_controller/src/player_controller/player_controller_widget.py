import os

from ament_index_python.resources import get_resource

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QHBoxLayout
from python_qt_binding.QtCore import Qt
from qt_material import apply_stylesheet

PLAYER_NUM = 5

class PlayerControllerWidget(QWidget):
  def __init__(self):
    super(PlayerControllerWidget, self).__init__()
    
    # �p�b�P�[�W���������ԈႦ�Ȃ��悤��
    pkg_name = 'player_controller'
    _, package_path = get_resource('packages', pkg_name)
    # UI�����[�h���邯�ǃt�@�C�������ԈႦ�Ȃ��悤��
    ui_file = os.path.join(
        package_path, 'share', pkg_name, 'resource', 'player_controller.ui')
    
    self._layout = QHBoxLayout()
    
    self._pwidgets = []
    
    for i in range(PLAYER_NUM):
      self._pwidgets.append(QWidget())
      loadUi(ui_file, self._pwidgets[-1]) # ������QWidget�C���X�^���X��ui�t�@�C�������[�h
      # self._pwidgets[-1].frmTeamColorDisp.setStyleSheet('background-color: magenta')
      self._layout.addWidget(self._pwidgets[-1])
      
    self.setLayout(self._layout)
    # apply_stylesheet(self, theme='light_blue.xml')
    
    # �I�u�W�F�N�g���͊ԈႦ�Ă������H������
    self.setObjectName('PlayerControllerWidget') 
    
  def setTeamColor(self, id, color):
    if color == 'cyan':
      self._pwidgets[id-1].ledtDispColorAndRole.setStyleSheet('background-color: cyan')
      
    elif color == 'magenta':
      self._pwidgets[id-1].ledtDispColorAndRole.setStyleSheet('background-color: magenta')
  
  def setRole(self, id, role):
    self._pwidgets[id-1].ledtDispColorAndRole.setText(role)
      