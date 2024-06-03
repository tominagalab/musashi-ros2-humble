import os

from ament_index_python.resources import get_resource

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout
from python_qt_binding.QtCore import Qt

class PlayerControllerWidget(QWidget):
  def __init__(self):
    super(PlayerControllerWidget, self).__init__()
    
    # �p�b�P�[�W���������ԈႦ�Ȃ��悤��
    pkg_name = 'player_controller'
    _, package_path = get_resource('packages', pkg_name)
    # UI�����[�h���邯�ǃt�@�C�������ԈႦ�Ȃ��悤��
    ui_file = os.path.join(
        package_path, 'share', pkg_name, 'resource', 'player_controller.ui')
    
    self._layout = QVBoxLayout()
    self.p1widget = QWidget()
    loadUi(ui_file, self.p1widget)
    self._layout.addWidget(self.p1widget)
    self.setLayout(self._layout)
    
    # �I�u�W�F�N�g���͊ԈႦ�Ă������H������
    self.setObjectName('PlayerControllerWidget') 