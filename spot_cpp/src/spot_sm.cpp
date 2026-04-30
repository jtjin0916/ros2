// spot_sm_ros2.cpp
//
// ROS2 port of spot_sm.cpp (mini_sm_node)
//
// What stays the same (원본과 동일한 로직):
// - teleop Twist 받아서 spot_mini.update_command()에 기록
// - estop Bool 토글/명령 0으로 만들기 + timeout 기준시간 갱신
// - switch_movement 서비스로 movement 모드 토글
// - 주기적으로 mini_cmd publish, timeout이면 Stop + 에러 로그
//
// TODO/POINT (QR 프로젝트에서 나중에 확인/수정):
// 1) 메시지 패키지명/헤더 경로: mini_ros::msg::MiniCmd -> qr_interfaces::msg::MiniCmd 등
// 2) spot.hpp/teleop.hpp 경로: mini_ros/spot.hpp -> qr_.../spot.hpp 등
// 3) 토픽명: "teleop", "estop", "mini_cmd" 네임스페이스(/qr/...) 적용 여부
// 4) 서비스명: "switch_movement" 네임스페이스 적용 여부
// 5) timeout/estop 동작 의도: 원본은 "estop.data==true" 메시지 들어올 때마다 토글(ON/OFF) 구조임
// 6) string 필드 지원: ROS2 msg에서 motion/movement가 string인지 확인(원본과 동일 가정)

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>

// ====== 프로젝트 구조에 맞게 수정 필요 (TODO/POINT) ======
#include <spot_cpp/spot.hpp>
#include <spot_cpp/teleop.hpp>
#include <spot_interfaces/msg/mini_cmd.hpp>
// =========================================================

using namespace std::chrono_literals;

// Global Vars (원본 유지)
spot::Spot spot_mini = spot::Spot();
bool teleop_flag = false;   // (원본에 있었지만 사용 안 함)
bool motion_flag = false;
bool ESTOP = false;

class MiniSMNode : public rclcpp::Node
{
public:
  MiniSMNode()
  : Node("mini_sm_node")
  {
    RCLCPP_INFO(this->get_logger(), "STARTING NODE: spot_mini State Machine (ROS2)");

    // Parameters
    frequency_ = this->declare_parameter<double>("frequency", 5.0);
    timeout_   = this->declare_parameter<double>("timeout", 1.0);  // 원본은 상수였는데 ROS2에서 param으로도 두면 편함

    // Init Publisher
    // TODO/POINT: msg 패키지명이 바뀌면 여기 타입도 같이 바꿔야 함
    mini_pub_ = this->create_publisher<spot_interfaces::msg::MiniCmd>("mini_cmd", 1);

    // Init Subscribers
    teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "teleop", 1,
      std::bind(&MiniSMNode::teleop_callback, this, std::placeholders::_1));

    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "estop", 1,
      std::bind(&MiniSMNode::estop_callback, this, std::placeholders::_1));

    // Init Service
    switch_movement_srv_ = this->create_service<std_srvs::srv::Empty>(
      "switch_movement",
      std::bind(&MiniSMNode::swm_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Init MiniCmd (원본 동일)
    mini_cmd_.x_velocity = 0.0;
    mini_cmd_.y_velocity = 0.0;
    mini_cmd_.rate = 0.0;
    mini_cmd_.roll = 0.0;
    mini_cmd_.pitch = 0.0;
    mini_cmd_.yaw = 0.0;
    mini_cmd_.z = 0.0;
    mini_cmd_.faster = 0.0;
    mini_cmd_.slower = 0.0;
    mini_cmd_.motion = "Stop";
    mini_cmd_.movement = "Stepping";

    last_time_ = this->get_clock()->now();
    current_time_ = last_time_;

    // Timer loop (frequency Hz)
    auto period = std::chrono::duration<double>(1.0 / std::max(0.1, frequency_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MiniSMNode::loop, this));
  }

private:
  void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr tw)
  {
    // 원본 동일: twist -> update_command
    spot_mini.update_command(
      tw->linear.x, tw->linear.y, tw->linear.z,
      tw->angular.z, tw->angular.x, tw->angular.y
    );
  }

  void estop_callback(const std_msgs::msg::Bool::SharedPtr estop)
  {
    // 원본은 estop.data == true가 들어올 때마다 토글하는 구조
    // TODO/POINT: estop 토픽이 "눌림 이벤트"인지 "상태 유지"인지 다시 확인 필요
    if (estop->data)
    {
      spot_mini.update_command(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      motion_flag = true;

      if (!ESTOP)
      {
        RCLCPP_ERROR(this->get_logger(), "ENGAGING MANUAL E-STOP!");
        ESTOP = true;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "DIS-ENGAGING MANUAL E-STOP!");
        ESTOP = false;
      }
    }

    last_time_ = this->get_clock()->now();
  }

  void swm_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
  {
    // 원본 동일: movement mode toggle
    spot_mini.switch_movement();
    motion_flag = true;
  }

  void loop()
  {
    current_time_ = this->get_clock()->now();

    spot::SpotCommand cmd = spot_mini.return_command();

    const double elapsed = (current_time_ - last_time_).seconds();

    // Condition for sending non-stop command (원본 동일)
    if (!motion_flag && !(elapsed > timeout_) && !ESTOP)
    {
      mini_cmd_.x_velocity = cmd.x_velocity;
      mini_cmd_.y_velocity = cmd.y_velocity;
      mini_cmd_.rate = cmd.rate;
      mini_cmd_.roll = cmd.roll;
      mini_cmd_.pitch = cmd.pitch;
      mini_cmd_.yaw = cmd.yaw;
      mini_cmd_.z = cmd.z;
      mini_cmd_.faster = cmd.faster;
      mini_cmd_.slower = cmd.slower;

      // Motion
      if (cmd.motion == spot::Go)
        mini_cmd_.motion = "Go";
      else
        mini_cmd_.motion = "Stop";

      // Movement
      if (cmd.movement == spot::Stepping)
        mini_cmd_.movement = "Stepping";
      else
        mini_cmd_.movement = "Viewing";
    }
    else
    {
      mini_cmd_.x_velocity = 0.0;
      mini_cmd_.y_velocity = 0.0;
      mini_cmd_.rate = 0.0;
      mini_cmd_.roll = 0.0;
      mini_cmd_.pitch = 0.0;
      mini_cmd_.yaw = 0.0;
      mini_cmd_.z = 0.0;
      mini_cmd_.faster = 0.0;
      mini_cmd_.slower = 0.0;
      mini_cmd_.motion = "Stop";
      // NOTE: 원본도 movement는 여기서 강제로 바꾸진 않음(Stop만 보냄)
    }

    if (elapsed > timeout_)
    {
      RCLCPP_ERROR(this->get_logger(), "TIMEOUT...ENGAGING E-STOP!");
    }

    mini_pub_->publish(mini_cmd_);
    motion_flag = false;
  }

private:
  // Params
  double frequency_{5.0};
  double timeout_{1.0};

  // ROS2 handles
  rclcpp::Publisher<spot_interfaces::msg::MiniCmd>::SharedPtr mini_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr switch_movement_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Time
  rclcpp::Time current_time_;
  rclcpp::Time last_time_;

  // Msg
  spot_interfaces::msg::MiniCmd mini_cmd_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MiniSMNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
