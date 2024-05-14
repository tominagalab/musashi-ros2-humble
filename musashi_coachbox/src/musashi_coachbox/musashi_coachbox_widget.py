import os

from ament_index_python.resources import get_resource

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import Qt

class CoachBoxWidget(QWidget):
  def __init__(self):
    super(CoachBoxWidget, self).__init__()
    
    pkg_name = 'musashi_coachbox'
    _, package_path = get_resource('packages', pkg_name)
    
    # UIをロードする
    # uiファイル名を間違えないように注意
    ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', 'coachbox_widget.ui')
    loadUi(ui_file, self)
    
    # オブジェクト名を設定する
    self.setObjectName('CoachBoxWidget')
    
  # この関数が一定周期で呼び出される
  def paintEvent(self, event):
    pass
