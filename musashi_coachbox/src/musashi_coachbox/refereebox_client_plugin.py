from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QTimer, Slot, Signal
import rclpy
import sys

from musashi_coachbox.refereebox_client_widget import RefereeBoxClientWidget

class RefereeBoxClientPlugin(Plugin):
  def __init__(self, context):
    super(RefereeBoxClientPlugin, self).__init__(context)
    
    # オブジェクト名を設定
    self.setObjectName('RefereeBoxClientPlugin')
    # contextをもらっておく（?）気にしないでいい
    self._context = context
    
    # ここでカスタムWidgetクラスの実態を作成する
    self._widget = RefereeBoxClientWidget() # 重要
    if context.serial_number() > 1:
      self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
      
    # contextにカスタムWidgetの追加（多分Pluginだからやっている）
    context.add_widget(self._widget)
    
    # カスタムWidgetを一定周期で更新するためのQTimerを作成
    self._timer = QTimer()
    
    # QTimerのtimeoutシグナルが発行されたらカスタムWidgetのupdateスロット関数を呼び出す
    # GUI画面が必要に応じて更新される
    self._timer.timeout.connect(self._widget.update)
    
    # タイマーを16msec周期で起動
    self._timer.start(16)
    
    
    
    
    