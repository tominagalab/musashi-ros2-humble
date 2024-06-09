import os

from ament_index_python.resources import get_resource

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QPainter, QFont
from python_qt_binding.QtCore import Qt
from qt_material import apply_stylesheet

class PlayerServerWidget(QWidget):
  def __init__(self):
    super(PlayerServerWidget, self).__init__()
    
    # パッケージ名も書き間違えないように
    pkg_name = 'musashi_rqt_player_server'
    _, package_path = get_resource('packages', pkg_name)
    # UIをロードするけどファイル名を間違えないように
    ui_file = os.path.join(
        package_path, 'share', pkg_name, 'resource', 'player_server_widget.ui')
    loadUi(ui_file, self)
    
    # apply_stylesheet(self, theme='light_blue.xml')

    # オブジェクト名は間違えても動く？未調査
    self.setObjectName('PlayerServerWidget') 