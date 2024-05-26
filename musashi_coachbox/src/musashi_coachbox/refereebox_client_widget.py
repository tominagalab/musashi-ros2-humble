import os

from ament_index_python.resources import get_resource

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import Qt

class RefereeBoxClientWidget(QWidget):
  def __init__(self):
    super(RefereeBoxClientWidget, self).__init__()
    
    pkg_name = 'musashi_coachbox'
    _, package_path = get_resource('packages', pkg_name)
    
    # UI�����[�h����
    # ui�t�@�C�������ԈႦ�Ȃ��悤�ɒ���
    ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', 'refereebox_widget.ui')
    loadUi(ui_file, self)
    
    # �I�u�W�F�N�g����ݒ肷��
    self.setObjectName('RefereeBoxClientWidget')
    
  # ���̊֐����������ŌĂяo�����
  def paintEvent(self, event):
    pass
