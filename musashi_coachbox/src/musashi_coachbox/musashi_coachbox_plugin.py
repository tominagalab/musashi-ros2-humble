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
    
    # カスタムWidgetを一定周期で更新するためのQTimer設定
    self._timer = QTimer()
    # QTimerのtimeoutシグナルが発行されたらカスタムWidgetのupdateスロット関数を呼び出す
    # GUI画面が必要に応じて更新される
    self._timer.timeout.connect(self._widget.update)
    
    # シグナルスロット接続
    self._widget.btnRefConnect.clicked.connect(self.onClickBtnRefConnect)    
    
    
    # RefBoxClient作成
    self._refbox_client = RefBoxClient()
    
    # 16msec周期で更新させる
    self._timer.start(16)
    
  def shutdown_plugin(self):
    # 終了時はタイマーを止める
    self._timer.stop()
    pass
  
  def save_settings(self, plugin_settings, instance_settings):
    # セーブ機能は何もしない
    # つらい気持ちをファイルに保存する機能　いる？
    pass
  
  def restore_settings(self, plugin_settings, instance_settings):
    # リストア機能は何もしない
    # つらい気持ちを復元してどうするの？
    pass
  
  @Slot()
  def onClickBtnRefConnect(self,):
    print(sys._getframe().f_code.co_name, ': ')
    self._refbox_client.connect()