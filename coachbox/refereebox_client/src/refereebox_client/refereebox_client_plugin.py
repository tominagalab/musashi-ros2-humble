from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QTimer

from refereebox_client.refereebox_client_widget import RefereeBoxClientWidget

class RefereeBoxClientPlugin(Plugin):
  def __init__(self, context):
    super(RefereeBoxClientPlugin, self).__init__(context)
    
    self.setObjectName('RefereeBoxClientPlugin')
    self._context = context
    
    self._widget = RefereeBoxClientWidget()
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
    pass    