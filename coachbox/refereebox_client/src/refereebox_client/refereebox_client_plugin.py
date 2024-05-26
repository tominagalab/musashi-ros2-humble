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
    
    # GUI�V�O�i���X���b�g�ڑ�
    self._widget.chckConnect.stateChanged.connect(lambda: self.onStateChangedChckConnect(self._widget.chckConnect.checkState()))
  
  def shutdown_plugin(self):
    # �I�����̓^�C�}�[���~�߂�
    self._timer.stop()
  
  def save_settings(self, plugin_settings, instance_settings):
    pass
  
  def restore_settings(self, plugin_settings, instance_settings):
    pass
  
  @Slot()
  def onStateChangedChckConnect(self, state):
    if state: # �`�F�b�N�����������ڑ�����
      # RefBoxClient�̐V�K�쐬
      self._refbox_client = RefBoxClient()
      # IP�A�h���X�ƃ|�[�g���擾
      refbox_address = self._widget.lnedtIP.text()
      refbox_port = int(self._widget.lnedtPort.text())
      # �ڑ��̃g���C
      isConnect = self._refbox_client.connect(refbox_address, refbox_port)
      
      if not isConnect: # ���s
        print('Connection error, please chech network condition')
        self._widget.chckConnect.setCheckState(False)
      else: # ����
        self._refbox_client.start()
    else: # �`�F�b�N���O�ꂽ���ؒf����
      # self._refbox_client.disconnect()
      # self._refbox_client.join()
      del self._refbox_client # �f�X�g���N�^�̌Ăяo��
      # python�ł͈ꉞ�����I�Ƀ���������������ۂ�