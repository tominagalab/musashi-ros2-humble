# coachbox
�R�[�`�{�b�N�X�֌W�̃p�b�P�[�W��z�u���Ă���f�B���N�g���ł��D
## �p�b�P�[�W���X�g  
- refereebox_client  
���t�F���[�{�b�N�X�ƒʐM���s���p�b�P�[�W�ł��D  
- player_server   
�e�v���C���[�ƒʐM���s���p�b�P�[�W�ł��D  

## RefereeBox-CoachBox�ԒʐM�ɂ���
RefereeBox�Ƃ�TCP�ő���M���s���܂��D  
### �R�}���h�t�H�[�}�b�g�ڍ�  
- RefereeBox�����JSON�`���̕�����f�[�^���o�C�i���f�[�^�ő����Ă��܂��D
- JSON�t�H�[�}�b�g�͈ȉ��̌`���������܂�  
```
{
  command: {�R�}���h������}
  targetTeam: {�^�[�Q�b�g�`�[��������}
}
```
- ��L�̕�����̖����ɂ͏I�[��\��'\0'�������đ����Ă��܂��D
- {�R�}���h������}�ɂ�"WELCOME"��"START"�Ȃǂ̕����񂪓���܂��D  
- {�^�[�Q�b�g�`�[��������}�ɂ�"224.16.32.*"�Ŋe�`�[���Ɋ���U���Ă���IP�A�h���X�̕�����f�[�^������܂��D���邢�� __��i""�j__ �̏ꍇ������܂��D 
  - ��i""�j�̏ꍇ�͎������̗��`�[�����ɑ����Ă��邱�Ƃ��Ӗ�����R�}���h�ɂȂ�܂��D  
  �i��jcommand��"START"��"DROP_BALL"�ł�targetTeam�͋�ł��D  
  - �^�[�Q�b�g�`�[��������i"224.16.32.*"�j�������Ă���ꍇ�́C���̃`�[���ɂ���R�}���h�ɂȂ�܂��D  
  �i��jcommand��"KICKOFF"�ŁCtargetTeam��"224.16.32.44"�̏ꍇ�́C�`�[��Hibikino-Musashi���L�b�N�I�t�ł��邱�Ƃ��Ӗ�����̂ŁC���`�[���̃L�b�N�I�t�|�W�V�����Ɉړ����C"START"�R�}���h��ҋ@����K�v������D 

### �R�}���h�ꗗ  
|command|targetTeam|  
|-------|----------|  
|"START"|""|  
|"STOP"|""|  
|"DROP_BALL"|""|  
|"HALF_TIME"|""|  
|"END_GAME"|""|  
|"GAME_OVER"|""|  
|"PARK"|""|  
|"FIRST_HALF"|""|  
|"SECOND_HALF"|""|  
|"FIRST_HALF_OVERTIME"|""|
|"SECOND_HALF_OVERTIME"|""|
|"RESET"|""|
|WELCOME|"224.16.32.*"|
|KICKOFF|"224.16.32.*"|
|FREEKICK|"224.16.32.*"|
|GOALKICK|"224.16.32.*"|
|THROWIN|"224.16.32.*"|
|CORNER|"224.16.32.*"|
|PENALTY|"224.16.32.*"|
|GOAL|"224.16.32.*"|
|REPAIR|"224.16.32.*"|
|YELLOW_CARD|"224.16.32.*"|
|DOUBLE_YELLOW|"224.16.32.*"|
|RED_CARD|"224.16.32.*"|
|SUBSTITUTION|"224.16.32.*"|
|IS_ALIVE|"224.16.32.*"|

## Hibkino-Musashi�ɂ�����CoachBox-Player�ԒʐM  
CoachBox��Player��UDP�ŒʐM���s���Ă��܂��D  
- ��̓I�ȒʐM�����ɂ��Ă�"musashi_player/communication/communication.cpp"���Q�Ƃ��Ă��������D  
  - UDP�̎�M�����ɂ��Ă�"recv"�֐��ōs���Ă��܂��D  
  - UDP�̑��M�����ɂ��Ă�"send"�֐��ōs���Ă��܂��D  
### CoachBox��Player�ւ̒ʐM  
CoachBox��RefereeBox���瑗���Ă����R�}���h�Ɋ�Â��āC�e�v���C���[�փR�}���h�𑗐M���܂��D  
���̎��CHibikino-Musashi���Ŏ�茈�߂�ꂽ�R�}���h�t�H�[�}�b�g�ɕϊ����đ���K�v������܂��D  
### Player��CoachBox�ւ̒ʐM  
�e�v���C���[����͊e�v���C���[�̏�ԃf�[�^���i�[���ꂽ������f�[�^�������Ă���D  
�i���o�C�i���f�[�^�ɂȂ��Ă��Ȃ����Ƃɒ��Ӂj  
#### �ʐM�t�H�[�}�b�g�@�@
�J���}�i","�j��؂�ňȉ��̏��ɐ���������������������ő����Ă���  

|index|value|detail|
|-----|-----|-----| 
|1|color|�`�[���J���[�DCYAN�Ȃ�0�CMAGENTA�Ȃ�1|
|2|id|���{�b�g��ID|
|3|action|���{�b�g�̃A�N�V�����iAction���O��Ԃ̒萔�l�j|
|4|state|���{�b�g�̃X�e�[�g�iState���O��Ԃ̒萔�l�j|
|5|ball.distance|�{�[���Ƃ̒�������|
|6|ball.angle|�{�[���̊p�x|
|7|goal.distance|�S�[���Ƃ̒�������|
|8|goal.angle|�S�[���̊p�x|
|9|myGoal.distance|���g�̃S�[���Ƃ̒�������|
|10|myGoal.angle|���g�̃S�[���̊p�x|
|11|position.x|���Ȉʒux���W|
|12|position.y|���Ȉʒuy���W|
|13|position.angle|�p����|
|14|role|���{�b�g�̃��[���iRole���O��Ԃ̒萔�l�j|
|15|haveBall|�{�[���ێ��̗L��|
|16|moveto_position.x|���{�b�g�̖ڕWx���W|
|17|moveto_position.y|���{�b�g�̖ڕWy���W|
|18|moveto_position.angle|���{�b�g�̖ڕW�p����|
|19|obstacle.distance|��Q���܂ł̒�������|
|20|obstacle.angle|��Q���̊p�x|

�R�[�`�{�b�N�X���ł́C��M��Ɂh,�h��split���ĕ����񂩂琮���l�ւ̕ϊ����K�v�ɂȂ�D  
��̕ϐ����������Ȃ̂���","��split����܂ł킩��Ȃ��D  
__������o�C�i���f�[�^�ő���M������@�ɏC�����ʐM���x�̍�������}��K�v������__  

