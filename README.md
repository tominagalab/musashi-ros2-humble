# musashi-ros2-humble
�{�p�b�P�[�W�� Robocup Hibikino-Musashi ��ROS2�p���|�W�g���ɂȂ�܂��D
Visual Studio��p����C++�݂̂̊J���Ɍ��E�������Ă������߁C
���t�F���[�{�b�N�X�̎d�l�ύX�ɍ��킹�ĐV����Ubuntu�x�[�X��Hibikino-musashi�Ƃ��ׂ��C�J�����J�n����D    

## �J����  
- Ubuntu 22.04  
- ROS2 humble  
- Visual Studio Code�i�G�f�B�^�j  
- python�CC++  

## �R�[�f�B���O�K��i�����K���j  
python�ł̃R�[�f�B���O�ɂ����Ă�PEP8�R�[�h�X�^�C���ɏ�������D
�ȉ����Q�Ƃ��邱�ƁD  
`https://peps.python.org/pep-0008/#code-lay-out`  
`https://atmarkit.itmedia.co.jp/ait/articles/2308/08/news020.html`  

## �f�B���N�g���\��     
<pre>
musashi-ros2-humble�i���[�g�f�B���N�g���j  
������ README.md
������ behavior
������ coachbox
������ hardware
������ localization
������ musashi_msgs
������ perception 
</pre>

|Name|Detail|  
|---|---|
|README.md|���̃t�@�C��|
|behavior|���[���x�[�X�s������C�X�e�[�g�}�V���C�s���I���̃f�B���N�g��|
|coachbox|���t�F���[�{�b�N�X�C�v���C���[�Ƃ̒ʐM�֌W�f�B���N�g��|
|hardware|�O���f�o�C�X����p�̃f�B���N�g��|
|localization|���Ȉʒu����p�̃f�B���N�g��|
|musashi_msgs|�Ǝ����b�Z�[�W�p�̃f�B���N�g��|
|perception|�O�E�F���C�m�o�n�̃f�B���N�g��|

## �p�b�P�[�W�쐬�R�}���h��  
- python�p�b�P�[�W  
``ros2 pkg create [package name] --build-type ament_python --dependencies rclpy``  
- C++�p�b�P�[�W  
``ros2 pkg create [package name] --build-type ament_cmake --dependencies rclcpp``  
- python��rqt�v���O�C���p�b�P�[�W  
`ros2 pkg create [package name] --build-type ament_python --dependencies rclpy python_qt_binding rqt_gui rqt_gui_py rqt_py_common`