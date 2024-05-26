from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QTimer, Slot

from refereebox_client.refereebox_client_widget import RefereeBoxClientWidget
from refereebox_client.refbox_client import RefBoxClient

from musashi_msgs.msg import RefereeCmd

class RefereeBoxClientPlugin(Plugin):
  def __init__(self, context):
    super(RefereeBoxClientPlugin, self).__init__(context)
    
    self.setObjectName('RefereeBoxClientPlugin')
    self._context = context
    self._node = context.node
    
    self._widget = RefereeBoxClientWidget()
    if context.serial_number() > 1:
      self._widget.setWindowTitle(
        self._widget.windowTitle() + (' (%d)' % context.serial_number()))
      context.add_widget(self._widget)  

    # GUIシグナルスロット接続
    self._widget.chckConnect.stateChanged.connect(self.onStateChangedChckConnect)
    
    # パブリッシャー作成
    self._pub_refcmd = self._node.create_publisher(RefereeCmd, '/referee_cmd', 5)
    
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
  
  def onStateChangedChckConnect(self, state):
    if state: # チェックが入った→接続処理
      # RefBoxClientの新規作成
      self._refbox_client = RefBoxClient()
      # IPアドレスとポートを取得
      refboxIP = self._widget.lnedtIP.text()
      refboxPort = int(self._widget.lnedtPort.text())
      
      # 接続のトライ
      self._node.get_logger().info('Try to connect to RefereeBox [{}:{}] ...'.format(refboxIP, refboxPort))
      isConnect = self._refbox_client.connect(refboxIP, refboxPort)
      
      if isConnect:
        self._node.get_logger().info('Successfully connected to RefereeBox')
        self._refbox_client.recievedCommand.connect(self.onRecievedCommand) # 受信時のシグナルスロット接続
        self._refbox_client.start() # RefereeBox client スレッドのスタート
      else:
        self._node.get_logger().error('Failed to connect to RefereeBox')
        self._node.get_logger().error('Please check network connection status')
        self._widget.chckConnect.setCheckState(False)
        self._refbox_client = None
    else: # チェックが外れた→切断処理
      # self._refbox_client.disconnect()
      # self._refbox_client.join()
      self._refbox_client = None # デストラクタの呼び出し
      # pythonでは一応自動的にメモリ解放されるっぽい
      
  def onRecievedCommand(self, command, targetTeam):
    self._node.get_logger().info('Recieved from RefereeBox')
    self._node.get_logger().info('command = {}, targetTeam = {}'.format(command, targetTeam))
    
    # メッセージ作成
    refereeCmd = RefereeCmd()
    # 値の代入
    refereeCmd.command = command
    refereeCmd.target_team = targetTeam
    
    # パブリッシュ
    self._pub_refcmd.publish(refereeCmd)
    