#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "musashi_movebase/msg/motor_state.msg"

using namespace std::chrono_literals;

/*
  MaxonEposManager�N���X
  EPOS�̐ڑ�����сC�^�C�}�R�[���o�b�N�����Ń��[�^�̌��݂̏�Ԃ��p�u���b�V������
  �m�[�h�̃N���X�ɂȂ�̂�Node�N���X���p��
*/
class MaxonEposManager : public rclcpp::Node {
  // �����o�ϐ���`
 private:
  rclcpp::TimerBase::SharedPtr _timer;  // �^�C�}

  // �����o�֐���`
 public:
  // �R���X�g���N�^
  MaxonEposManager() : Node("maxon_epos_manager") {
    // �^�C�}���Ԃ̍쐬�Dcreate_wall_timer��Node�N���X�i�e�N���X�j�������Ă���
    _timer = this->create_wall_timer(
        250ms, std::bind(&MaxonEposManager::timer_callback, this));
  }

 private:
  void timer_callback() { RCLCPP_INFO(this->get_logger(), "callback"); }
};

/*
  main�֐�
*/
int main(int argc, char *argv[]) {
  // ������
  rclcpp::init(argc, argv);
  // spin�֐���Node�I�u�W�F�N�g��n���ăm�[�h�����s
  rclcpp::spin(std::make_shared<MaxonEposManager>());
  // spin�֐����I��������V���b�g�_�E������
  rclcpp::shutdown();
  return 0;
}