from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QTimer, Slot

from player_controller.player_controller_widget import PlayerControllerWidget

class PlayerControllerPlugin(Plugin):
  def __init__(self, context):
    super(PlayerControllerPlugin, self).__init__(context)
    
    self.setObjectName('PlayerControllerPlugin')
    self._context = context
    self._node = context.node
    
    # ウィジェット実態の作成
    self._widget = PlayerControllerWidget()
    
    # 複数出してしまった時に個別のインスタンスが作成されるように一工夫
    if context.serial_number() > 1:
      self._widget.setWindowTitle(
        self._widget.windowTitle() + (' (%d)' % context.serial_number()))
    
    context.add_widget(self._widget)
    
    self._widget.setTeamColor(3, 'cyan')
    self._widget.setRole(3, 'alpha')
          
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