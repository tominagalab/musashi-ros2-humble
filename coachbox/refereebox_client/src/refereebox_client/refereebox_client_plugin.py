from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QTimer, Slot

from refereebox_client.refereebox_client_widget import RefereeBoxClientWidget
from refereebox_client.refbox_client import RefBoxClient

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
    
    # GUIシグナルスロット接続
    self._widget.chckConnect.stateChanged.connect(lambda: self.onStateChangedChckConnect(self._widget.chckConnect.checkState()))
  
  def shutdown_plugin(self):
    # 終了時はタイマーを止める
    self._timer.stop()
  
  def save_settings(self, plugin_settings, instance_settings):
    pass
  
  def restore_settings(self, plugin_settings, instance_settings):
    pass
  
  @Slot()
  def onStateChangedChckConnect(self, state):
    if state: # チェックが入った→接続処理
      # RefBoxClientの新規作成
      self._refbox_client = RefBoxClient()
      # IPアドレスとポートを取得
      refbox_address = self._widget.lnedtIP.text()
      refbox_port = int(self._widget.lnedtPort.text())
      # 接続のトライ
      isConnect = self._refbox_client.connect(refbox_address, refbox_port)
      
      if not isConnect: # 失敗
        print('Connection error, please chech network condition')
        self._widget.chckConnect.setCheckState(False)
      else: # 成功
        self._refbox_client.start()
    else: # チェックが外れた→切断処理
      # self._refbox_client.disconnect()
      # self._refbox_client.join()
      del self._refbox_client # デストラクタの呼び出し
      # pythonでは一応自動的にメモリ解放されるっぽい