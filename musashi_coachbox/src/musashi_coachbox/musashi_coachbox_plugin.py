# coding: UTF-8
# このpythonモジュールがカスタムPluginの定義になります

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QTimer, Slot, Signal
import rclpy
import sys

from musashi_coachbox.musashi_coachbox_widget import CoachBoxWidget
from musashi_coachbox.refbox_client import RefBoxClient

# クラス名はplugin.xmlから参照するのでスペルミスに注意
class CoachBoxPlugin(Plugin):
  def __init__(self, context):
    super(CoachBoxPlugin, self).__init__(context)
    
    # オブジェクト名を設定
    self.setObjectName('CoachBoxPlugin')
    # contextをもらっておく（?）気にしないでいい
    self._context = context
    
    # ここでカスタムWidgetクラスの実態を作成する
    self._widget = CoachBoxWidget()
    
    if context.serial_number() > 1:
        self._widget.setWindowTitle(
            self._widget.windowTitle() + (' (%d)' % context.serial_number()))
    
    # contextにカスタムWidgetの追加（多分Pluginだからやっている）
    context.add_widget(self._widget)
    
    # カスタムWidgetを一定周期で更新するためのQTimerを作成
    self._timer = QTimer()
    
    # QTimerのtimeoutシグナルが発行されたらカスタムWidgetのupdateスロット関数を呼び出す
    # GUI画面が必要に応じて更新される
    self._timer.timeout.connect(self._widget.update)
    
    # GUIシグナルスロット接続
    self._widget.chckRefConnect.stateChanged.connect(lambda: self.onStateChangedChckRefConnect(self._widget.chckRefConnect.checkState()))
    # self._widget.btnRefConnect.clicked.connect(self.onClickBtnRefConnect)    
    # self._widget.btnRefDisconnect.clicked.connect(self.onClickBtnRefDisconnect)    
        
    # タイマーを16msec周期で起動
    self._timer.start(16)
    
  def shutdown_plugin(self):
    # 終了時はタイマーを止める
    self._timer.stop()
    pass
  
  def save_settings(self, plugin_settings, instance_settings):
    pass
  
  def restore_settings(self, plugin_settings, instance_settings):
    pass
  
  @Slot()
  def onStateChangedChckRefConnect(self, state):
    if state: # チェックが入った→接続処理
      # RefBoxClientの新規作成
      self._refbox_client = RefBoxClient()
      # IPアドレスとポートを取得
      refbox_address = self._widget.lnedtRefAddress.text()
      refbox_port = int(self._widget.lnedtRefPort.text())
      # 接続のトライ
      isConnect = self._refbox_client.connect(refbox_address, refbox_port)
      
      if not isConnect: # 失敗
        print('Connection error, please chech network condition')
      else: # 成功
        self._refbox_client.start()
    else: # チェックが外れた→切断処理
      self._refbox_client.disconnect()
      self._refbox_client.join()
      del self._refbox_client # デストラクタの呼び出し
      # pythonでは一応自動的にメモリ解放されるっぽい
  
  # @Slot()
  # def onClickBtnRefConnect(self,):

  #   # RefBoxClientの新規作成
  #   self._refbox_client = RefBoxClient()
    
  #   refbox_address = self._widget.lnedtRefAddress.text()
  #   refbox_port = int(self._widget.lnedtRefPort.text())
  #   isConnect = self._refbox_client.connect(refbox_address, refbox_port)
    
  #   if not isConnect: 
  #     print('Connection error, please chech network condition')
  #   else:
  #     self._refbox_client.start()
  #     self._widget.btnRefDisconnect.setEnabled(True)
  #     self._widget.btnRefConnect.setEnabled(False)
  
  # @Slot()
  # def onClickBtnRefDisconnect(self,):
  #   self._refbox_client.disconnect()
  #   self._refbox_client.join()
  #   del self._refbox_client # デストラクタの呼び出し
  #   # pythonでは一応自動的にメモリ解放されるっぽい
  #   self._widget.btnRefDisconnect.setEnabled(False)
  #   self._widget.btnRefConnect.setEnabled(True)