from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QTimer, Slot

from refereebox_client.refereebox_client_widget import RefereeBoxClientWidget
from player_server.player_server_widget import PlayerServerWidget

from musashi_msgs.msg import RefereeCmd

class PlayerServerPlugin(Plugin):
  def __init__(self, context):
    super(PlayerServerPlugin, self).__init__(context)
    
    self.setObjectName('PlayerServerPlugin')
    self._context = context
    self._node = context.node
    
    self._widget = PlayerServerWidget()
    if context.serial_number() > 1:
      self._widget.setWindowTitle(
        self._widget.windowTitle() + (' (%d)' % context.serial_number()))
      context.add_widget(self._widget)
      
    self._timer = QTimer()
    self._timer.timeout.connect(self._widget.update)
    self._timer.start(16)
    
  def shutdown_plugin(self):
    # 終了時はタイマーを止める
    self._timer.stop()
  
  def save_settings(self, plugin_settings, instance_settings):
    pass
  
  def restore_settings(self, plugin_settings, instance_settings):
    pass
  
  