#ifndef TELEOP_INCLUDE_GUARD_HPP
#define TELEOP_INCLUDE_GUARD_HPP
/// \file
/// \brief Teleoperation Library that converts Joystick commands to motion

#include <vector>  // (원본 유지, 실제로 안 쓰면 제거 가능)

// ROS2 msg headers
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

// TODO/POINT: QR 프로젝트 메시지 패키지로 교체 가능
#include <spot_interfaces/msg/joy_buttons.hpp>
// 예: #include <qr_interfaces/msg/joy_buttons.hpp>

namespace tele
{
    /// \brief Teleop class responsible for converting Joystick commands into linear and angular velocity
    class Teleop
    {
    public:
        Teleop(const int & linear_x, const int & linear_y, const int & linear_z,
               const int & angular, const double & l_scale, const double & a_scale,
               const int & LB, const int & RB, const int & B_scale, const int & LT,
               const int & RT, const int & UD, const int & LR,
               const int & sw, const int & es);

        /// \brief Takes a Joy message and converts it to linear and angular velocity (Twist)
        /// \param joy: sensor_msgs describing Joystick inputs
        ///
        /// ROS1 -> ROS2 change:
        /// - ROS1: void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        /// - ROS2: void joyCallback(const sensor_msgs::msg::Joy& joy);
        void joyCallback(const sensor_msgs::msg::Joy & joy);

        /// \brief returns the most recently commanded Twist
        geometry_msgs::msg::Twist return_twist() const;

        /// \brief returns whether the movement switch trigger has been pressed
        bool return_trigger() const;

        /// \brief returns whether the E-STOP has been pressed
        bool return_estop() const;

        /// \brief returns other joystick button triggers (arrow pad etc)
        /// TODO/POINT: msg 패키지명이 바뀌면 반환 타입도 같이 변경
        spot_interfaces::msg::JoyButtons return_buttons() const;

    private:
        // AXES ON JOYSTICK
        int linear_x_ = 0;
        int linear_y_ = 0;
        int linear_z_ = 0;
        int angular_ = 0;
        int RB_ = 0;
        int LB_ = 0;

        // BUTTONS ON JOYSTICK
        int sw_ = 0;
        int es_ = 0;
        int RT_ = 0;
        int LT_ = 0;
        int UD_ = 0;
        int LR_ = 0;

        // AXIS SCALES
        double l_scale_ = 1.0;
        double a_scale_ = 1.0;
        double B_scale_ = 1.0;

        // TWIST
        geometry_msgs::msg::Twist twist;

        // TRIGGERS
        bool switch_trigger = false;
        bool ESTOP = false;

        // Arrow pad axes (ROS1 int였지만 실제는 float axis 값이 들어올 수 있음)
        // TODO/POINT: JoyButtons msg 타입이 float인지 int인지에 맞춰 조정 필요
        double updown = 0.0;
        double leftright = 0.0;

        bool left_bump = false;
        bool right_bump = false;
    };

}  // namespace tele

#endif  // TELEOP_INCLUDE_GUARD_HPP
