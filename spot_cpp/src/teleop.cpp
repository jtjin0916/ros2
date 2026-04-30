#include "spot_cpp/teleop.hpp"   // TODO/POINT: 헤더 경로를 qr_control 구조에 맞게 수정

// ROS2 message headers
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

// TODO/POINT: QR 프로젝트 메시지 패키지로 교체
#include <spot_interfaces/msg/joy_buttons.hpp>
// 예: #include <qr_interfaces/msg/joy_buttons.hpp>

namespace tele
{
  Teleop::Teleop(const int & linear_x, const int & linear_y, const int & linear_z,
                 const int & angular, const double & l_scale, const double & a_scale,
                 const int & LB, const int & RB, const int & B_scale, const int & LT,
                 const int & RT, const int & UD, const int & LR,
                 const int & sw, const int & es)
  {
    linear_x_ = linear_x;
    linear_y_ = linear_y;
    linear_z_ = linear_z;
    angular_ = angular;

    l_scale_ = l_scale;
    a_scale_ = a_scale;

    RB_ = RB;
    LB_ = LB;
    B_scale_ = B_scale;

    RT_ = RT;
    LT_ = LT;

    sw_ = sw;
    es_ = es;

    UD_ = UD;
    LR_ = LR;

    switch_trigger = false;
    ESTOP = false;
    updown = 0.0;
    leftright = 0.0;

    left_bump = false;
    right_bump = false;

    // Initialize twist to 0 (optional but safe)
    twist = geometry_msgs::msg::Twist();
  }

  // ROS2: take message by const reference (추천)
  void Teleop::joyCallback(const sensor_msgs::msg::Joy & joy)
  {
    // NOTE: no bounds checking, same as original.
    // TODO/POINT: axis/button index가 패드마다 다르니 파라미터로 맞춘 값이 유효한지 확인.

    twist.linear.x = l_scale_ * joy.axes[linear_x_];
    twist.linear.y = l_scale_ * joy.axes[linear_y_];
    // NOTE: used to control robot height
    twist.linear.z = -l_scale_ * joy.axes[linear_z_];

    twist.angular.z = a_scale_ * joy.axes[angular_];

    // NOTE: bottom bumpers used for changing step velocity
    twist.angular.x = B_scale_ * joy.axes[RB_];
    twist.angular.y = B_scale_ * joy.axes[LB_];

    // Switch Trigger: Button A
    switch_trigger = static_cast<bool>(joy.buttons[sw_]);

    // ESTOP: Button B
    ESTOP = static_cast<bool>(joy.buttons[es_]);

    // Arrow Pad
    updown = joy.axes[UD_];
    leftright = -joy.axes[LR_];

    // Top Bumpers
    left_bump = static_cast<bool>(joy.buttons[LT_]);
    right_bump = static_cast<bool>(joy.buttons[RT_]);
  }

  geometry_msgs::msg::Twist Teleop::return_twist() const
  {
    return twist;
  }

  bool Teleop::return_trigger() const
  {
    return switch_trigger;
  }

  bool Teleop::return_estop() const
  {
    return ESTOP;
  }

  // TODO/POINT: msg 패키지명/네임스페이스를 QR로 바꾸면 여기 반환 타입도 바꿔야 함.
  spot_interfaces::msg::JoyButtons Teleop::return_buttons() const
  {
    spot_interfaces::msg::JoyButtons jb;
    jb.updown = updown;
    jb.leftright = leftright;
    jb.left_bump = left_bump;
    jb.right_bump = right_bump;
    return jb;
  }

}  // namespace tele
