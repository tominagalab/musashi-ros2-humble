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
      
    self._timer = QTimer()
    self._timer.timeout.connect(self._widget.update)
    self._timer.start(16)
    
    # GUI�V�O�i���X���b�g�ڑ�
    self._widget.chckConnect.stateChanged.connect(self.onStateChangedChckConnect)
    
    # �p�u���b�V���[�쐬
    self._pub_refcmd = self._node.create_publisher(RefereeCmd, '/referee_cmd', 5)
  
  def shutdown_plugin(self):
    # �I�����̓^�C�}�[���~�߂�
    self._timer.stop()
  
  def save_settings(self, plugin_settings, instance_settings):
    pass
  
  def restore_settings(self, plugin_settings, instance_settings):
    pass
  
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
        self._refbox_client = None
      else: # ����
        self._refbox_client.recievedCommand.connect(self.onRecievedCommand) # ��M���̃V�O�i���X���b�g�ڑ�
        self._refbox_client.start() # RefereeBox client �X���b�h�̃X�^�[�g
    else: # �`�F�b�N���O�ꂽ���ؒf����
      # self._refbox_client.disconnect()
      # self._refbox_client.join()
      self._refbox_client = None # �f�X�g���N�^�̌Ăяo��
      # python�ł͈ꉞ�����I�Ƀ���������������ۂ�
      
  def onRecievedCommand(self, command, targetTeam):
    print(command, targetTeam)
    
    # ���b�Z�[�W�쐬
    refereeCmd = RefereeCmd()
    # �l�̑��
    refereeCmd.command = command
    refereeCmd.targat_team = targetTeam
    
    # �p�u���b�V��
    self._pub_refcmd.publish(refereeCmd)
    