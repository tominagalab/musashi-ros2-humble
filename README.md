# musashi-ros2-humble
�{�p�b�P�[�W�� Robocup Hibikino-Musashi ��ROS2�p���|�W�g���ɂȂ�܂��D
Visual Studio��p����C++�݂̂̊J���Ɍ��E�������Ă������߁C
���t�F���[�{�b�N�X�̎d�l�ύX�ɍ��킹�ĐV����Ubuntu�x�[�X��Hibikino-musashi�Ƃ��ׂ��C�J�����J�n����D    

## �J����  
- Ubuntu 22.04  
- ROS2 humble  
- Visual Studio Code�i�G�f�B�^�j  
- python�CC++  

## �J���K��  
PEP8�R�[�h�X�^�C���ɏ�������D  
`https://peps.python.org/pep-0008/#code-lay-out`  

## �f�B���N�g���\��   
<pre>
musashi-ros2-humble�i���[�g�f�B���N�g���j  
|-- README.md�i���̃t�@�C���j  
|-- behavior�i���[���x�[�X�s���I���C�ӎv����p�b�P�[�W�p�f�B���N�g���j  
|-- coachbox�i�R�[�`�{�b�N�X�֘A�p�b�P�[�W�p�f�B���N�g���j  
|-- hardware�i�O���f�o�C�X�֘A�p�b�P�[�W�p�f�B���N�g���j    
|-- localization�i���Ȉʒu����֘A�p�b�P�[�W�p�f�B���N�g���j  
|-- musashi_msgs�i�Ǝ����b�Z�[�W�p�f�B���N�g���j  
`-- perception�i�O�E�F�����C�m�o�n�p�b�P�[�W�p�f�B���N�g���j  
</pre>

## �p�b�P�[�W�쐬�R�}���h��  
### python�p�b�P�[�W�@�@ 
``ros2 pkg create [package name] --build-type ament_python --dependencies rclpy``  
### C++�p�b�P�[�W�@�@
``ros2 pkg create [package name] --build-type ament_cmake --dependencies rclcpp``  