# musashi_coachbox  
�R�[�`�{�b�N�X�p��ROS2�p�b�P�[�W.  
rqt�̃v���O�C���Ƃ��ĊJ�����܂��D  
## �Q�l����  
RQt�̃v���O�C���Ƃ��ĊJ������K�v������̂ŏ��X�J���̎d������₱�����D  
��������Q�l�� �� https://zenn.dev/shotaak/articles/f698974b59a02d  
�Q�l�ɂȂ�github���|�W�g���͈ȉ��D  
- rqt_topic���|�W�g���F https://github.com/ros-visualization/rqt_topic/tree/humble  
python�p�b�P�[�W�Ƃ��č쐬����Ă��܂��D  
- rqt_console���|�W�g���F https://github.com/ros-visualization/rqt_console/tree/dashing-devel  
CMake�p�b�P�[�W�Ƃ��č쐬����Ă��܂��D  
## �J����  
### Qt Designer  
rqt�v���O�C����GUI��ʂ���邽�߂̎g���₷���G�f�B�^�ł��D  
#### �C���X�g�[�����@  
�ȉ��̃R�}���h�ŕK�v�ȃp�b�P�[�W���C���X�g�[�����邾���D   
QtCreator��QtDesigner���܂܂�Ă��邾���Ȃ̂ŁCQtCreator�͎g��Ȃ��Ă����D  
```sudo apt install qtbase5-dev clang qtcreator```  
## �J�����@  
�ȉ��̃t�@�C����ҏW���J�����Ă����܂��D  
1. `resource/musashi_coachbox.ui`  
1. `src/musashi_coachbox/musashi_coachbox_widget.py`  
1. `src/musashi_coachbox/musashi_coachbox.py`  
### musashi_coachbox.ui  
�e�L�X�g�{�b�N�X���{�^�����CUI�̒�`�t�@�C���Œ��g��XML�ŋL�q����Ă��܂��D  
resource�f�B���N�g���ɒu���Ă��܂�.  
�J���E�ҏW��Qt Designer��.ui�t�B�A�����J���čs���܂��D  
XML�����׋�����K�v�͂���܂���D
### musashi_coachbox_widget.py  
QWidget�N���X���p�������J�X�^��Widget�N���X�̃��W���[���ł��D
src/musashi_coachbox�f�B���N�g���ɒu���Ă��܂��D  
�J�X�^��Widget�N���X���������`����Ă���C���̒���.ui�t�@�C�������[�h���ĉ�ʂ�`�悵�Ă��܂��D
### musashi_coachbox.py  
Plugin�N���X���p�������J�X�^��Plugin�N���X�̃��W���[���ł��D  
src/musashi_coachbox�f�B���N�g���ɒu���Ă��܂�.  
�J�X�^��Widget�N���X����`���ꂽ���W���[����import���Ă���CPlugin�Ƃ��ċN������@�\�����L���Ă��܂��D
rqt�͂����炭������1�ԍŏ��ɓǂݍ���Ŏ��s���Ă���͂��D
### ���̑��@�\���ɊJ���������W���[��  
#### refbox_client.py  
RefereeBox�Ƃ̒ʐM��S������RefBoxClient�N���X���������ꂽ���W���[���D  
musashi_coachbox.py����g�p���܂��D  
## RefereeBox�Ƃ̒ʐM�ɂ���  
RefereeBox�Ƃ�TCP�ʐM�ŃR�}���h�̂���肪�s����D(UDP�ł͂Ȃ��̂ŊԈႦ�Ȃ��悤�ɁI)  
�܂��C�ʐM��json�`���̃e�L�X�g�f�[�^�ōs����D  
### �ڑ��V�[�P���X  
�i�V�[�P���X�}�����̂����쐬���܂��D�j
### ���̑�  
## package.xml  
�p�b�P�[�W�̈ˑ��֌W��ݒ肵�Ă���xml�t�@�C���D
## plugin.xml  
�v���O�C���Ƃ��Ă̐U�镑����ݒ肵�Ă���xml�t�@�C���D  
