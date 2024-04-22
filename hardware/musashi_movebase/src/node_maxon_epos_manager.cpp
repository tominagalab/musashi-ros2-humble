#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "musashi_movebase/msg/motor_state.msg"

using namespace std::chrono_literals;

/*
  MaxonEposManagerクラス
  EPOSの接続および，タイマコールバック処理でモータの現在の状態をパブリッシュする
  ノードのクラスになるのでNodeクラスを継承
*/
class MaxonEposManager : public rclcpp::Node {
  // メンバ変数定義
 private:
  rclcpp::TimerBase::SharedPtr _timer;  // タイマ

  // メンバ関数定義
 public:
  // コンストラクタ
  MaxonEposManager() : Node("maxon_epos_manager") {
    // タイマ実態の作成．create_wall_timerはNodeクラス（親クラス）が持っている
    _timer = this->create_wall_timer(
        250ms, std::bind(&MaxonEposManager::timer_callback, this));
  }

 private:
  void timer_callback() { RCLCPP_INFO(this->get_logger(), "callback"); }
};

/*
  main関数
*/
int main(int argc, char *argv[]) {
  // 初期化
  rclcpp::init(argc, argv);
  // spin関数にNodeオブジェクトを渡してノードを実行
  rclcpp::spin(std::make_shared<MaxonEposManager>());
  // spin関数が終了したらシャットダウン処理
  rclcpp::shutdown();
  return 0;
}