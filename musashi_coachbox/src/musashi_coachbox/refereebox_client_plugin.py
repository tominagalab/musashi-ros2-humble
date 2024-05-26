from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QTimer, Slot, Signal
import rclpy
import sys

from musashi_coachbox.refereebox_client_widget import RefereeBoxClientWidget

class RefereeBoxClientPlugin(Plugin):
  def __init__(self, context):
    super(RefereeBoxClientPlugin, self).__init__(context)
    
    # �I�u�W�F�N�g����ݒ�
    self.setObjectName('RefereeBoxClientPlugin')
    # context��������Ă����i?�j�C�ɂ��Ȃ��ł���
    self._context = context
    
    # �����ŃJ�X�^��Widget�N���X�̎��Ԃ��쐬����
    self._widget = RefereeBoxClientWidget() # �d�v
    if context.serial_number() > 1:
      self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
      
    # context�ɃJ�X�^��Widget�̒ǉ��i����Plugin���������Ă���j
    context.add_widget(self._widget)
    
    # �J�X�^��Widget���������ōX�V���邽�߂�QTimer���쐬
    self._timer = QTimer()
    
    # QTimer��timeout�V�O�i�������s���ꂽ��J�X�^��Widget��update�X���b�g�֐����Ăяo��
    # GUI��ʂ��K�v�ɉ����čX�V�����
    self._timer.timeout.connect(self._widget.update)
    
    # �^�C�}�[��16msec�����ŋN��
    self._timer.start(16)
    
    
    
    
    