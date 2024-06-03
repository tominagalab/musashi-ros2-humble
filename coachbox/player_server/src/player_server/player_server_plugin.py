from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QTimer, Slot

from player_server.player_server_widget import PlayerServerWidget
from player_server.player_server import PlayerServer

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
          
    # サブスクライバー作成
    self._sub_refcmd = self._node.create_subscription(
      RefereeCmd,
      '/referee_cmd',
      self.refcmd_callback,
      10
    )
    
    # シグナル-スロット接続
    self._player_server = PlayerServer()
    self._player_server.recievedPlayerData.connect(self.onRecievedPlayerData)
      
    self._timer = QTimer()
    self._timer.timeout.connect(self._widget.update)
    self._timer.start(16)
    
    self._player_server.open()
    self._player_server.start()
    
  def shutdown_plugin(self):
    # 終了時はタイマーを止める
    self._timer.stop()
    self._player_server.close()
  
  def save_settings(self, plugin_settings, instance_settings):
    pass
  
  def restore_settings(self, plugin_settings, instance_settings):
    pass
  
  def refcmd_callback(self, msg):
    print(msg.command, msg.target_team)
  
  @Slot(int,dict)
  def onRecievedPlayerData(self, id, data):
    self._node.get_logger().info(id, data)
  