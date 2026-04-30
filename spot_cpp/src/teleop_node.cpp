// teleop_node_ros2.cpp
//
// ROS2 port of teleop_node.cpp
//
// Keeps original behavior as close as possible:
// - Reads joystick via tele::Teleop::joyCallback (Teleop class owns joystick parsing/state)
// - Publishes:
//    * "teleop"   (geometry_msgs::msg::Twist)
//    * "estop"    (std_msgs::msg::Bool)
//    * "joybuttons" (mini_ros::msg::JoyButtons)  <-- TODO/POINT: QR msg pkg로 바꿀 것
// - Calls service "switch_movement" (std_srvs::srv::Empty) with debounce
//
// TODO/POINT (QR 프로젝트에서 나중에 확인/수정):
// 1) 메시지 패키지명: mini_ros::msg::JoyButtons -> qr_interfaces::msg::JoyButtons 등
// 2) teleop.hpp include 경로: mini_ros/teleop.hpp -> qr_* 로 교체
// 3) joy topic QoS: sensor_msgs/Joy는 보통 BEST_EFFORT가 적합할 수 있음
// 4) debounce_thresh와 "trigger(=switch_movement)" 의도 확인
// 5) switch_movement 서비스 네임스페이스(/qr/...) 적용 여부

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/joy.hpp>

// ====== 프로젝트 구조에 맞게 수정 필요 (TODO/POINT) ======
#include <spot_cpp/teleop.hpp>
#include <spot_interfaces/msg/joy_buttons.hpp>
// =========================================================

using namespace std::chrono_literals;

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode()
  : Node("teleop_node")
  {
    RCLCPP_INFO(this->get_logger(), "STARTING NODE: Teleoperation (ROS2)");

    // -------------------------
    // Parameters (same names as ROS1)
    // -------------------------
    frequency_ = this->declare_parameter<double>("frequency", 60.0);

    axis_linear_x_ = this->declare_parameter<int>("axis_linear_x", 3);
    axis_linear_y_ = this->declare_parameter<int>("axis_linear_y", 2);
    axis_linear_z_ = this->declare_parameter<int>("axis_linear_z", 1);
    axis_angular_  = this->declare_parameter<int>("axis_angular", 0);

    scale_linear_  = this->declare_parameter<double>("scale_linear", 1.0);
    scale_angular_ = this->declare_parameter<double>("scale_angular", 1.0);
    scale_bumper_  = this->declare_parameter<double>("scale_bumper", 1.0);

    button_switch_ = this->declare_parameter<int>("button_switch", 1);
    button_estop_  = this->declare_parameter<int>("button_estop", 2);

    rb_ = this->declare_parameter<int>("rb", 9);
    lb_ = this->declare_parameter<int>("lb", 8);
    rt_ = this->declare_parameter<int>("rt", 7);
    lt_ = this->declare_parameter<int>("lt", 6);
    updown_ = this->declare_parameter<int>("updown", 5);
    leftright_ = this->declare_parameter<int>("leftright", 4);

    debounce_thresh_ = this->declare_parameter<double>("debounce_thresh", 0.15);

    // -------------------------
    // Teleop core object (same ctor signature as ROS1)
    // -------------------------
    teleop_ = std::make_shared<tele::Teleop>(
      axis_linear_x_, axis_linear_y_, axis_linear_z_, axis_angular_,
      scale_linear_, scale_angular_,
      lb_, rb_, scale_bumper_,
      lt_, rt_,
      updown_, leftright_,
      button_switch_, button_estop_
    );

    // -------------------------
    // Publishers
    // -------------------------
    estop_pub_ = this->create_publisher<std_msgs::msg::Bool>("estop", 1);
    vel_pub_   = this->create_publisher<geometry_msgs::msg::Twist>("teleop", 1);
    jb_pub_    = this->create_publisher<spot_interfaces::msg::JoyButtons>("joybuttons", 1);

    // -------------------------
    // Service client
    // -------------------------
    switch_movement_client_ = this->create_client<std_srvs::srv::Empty>("switch_movement");

    // Wait for service (similar to ROS1 waitForService)
    // NOTE: This blocks startup until available; you can remove if you prefer non-blocking.
    while (!switch_movement_client_->wait_for_service(1s) && rclcpp::ok()) {
      RCLCPP_WARN(this->get_logger(), "Waiting for service: switch_movement ...");
    }

    // -------------------------
    // Subscriber: joy
    // -------------------------
    // TODO/POINT: For joystick, BEST_EFFORT often makes sense (drops are OK).
    // Keeping default QoS depth=1 here to mirror ROS1 queue_size=1.
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy",
      rclcpp::QoS(1),
      std::bind(&TeleopNode::joy_callback, this, std::placeholders::_1)
    );

    // -------------------------
    // Timing
    // -------------------------
    last_time_ = this->get_clock()->now();

    // Main loop timer (frequency Hz)
    auto period = std::chrono::duration<double>(1.0 / std::max(0.1, frequency_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TeleopNode::loop, this)
    );
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // ROS1: teleop::joyCallback was a member fn taking JoyConstPtr.
    // ROS2: pass dereferenced message (assuming Teleop class accepts const sensor_msgs::msg::Joy&).
    //
    // TODO/POINT: Teleop::joyCallback 시그니처를 ROS2 msg 타입으로 바꿔야 함.
    //   - ROS1: void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    //   - ROS2: void joyCallback(const sensor_msgs::msg::Joy& joy);   (추천)
    //
    // 여기서는 "teleop_"가 ROS2 시그니처로 이미 바뀌었다고 가정하고 호출.
    teleop_->joyCallback(*msg);
  }

  void loop()
  {
    const auto now = this->get_clock()->now();
    const double elapsed = (now - last_time_).seconds();

    std_msgs::msg::Bool estop;
    estop.data = teleop_->return_estop();

    if (estop.data && elapsed >= debounce_thresh_)
    {
      RCLCPP_INFO(this->get_logger(), "SENDING E-STOP COMMAND!");
      last_time_ = now;
    }
    else if (!teleop_->return_trigger())
    {
      // Send Twist
      vel_pub_->publish(teleop_->return_twist());
      estop.data = false;
    }
    else if (elapsed >= debounce_thresh_)
    {
      // Call Switch Service
      auto req = std::make_shared<std_srvs::srv::Empty::Request>();
      // async call; we don't need the response content
      (void)switch_movement_client_->async_send_request(req);

      estop.data = false;
      last_time_ = now;
    }

    // Publish buttons every cycle
    jb_pub_->publish(teleop_->return_buttons());

    // Publish estop every cycle
    estop_pub_->publish(estop);
  }

private:
  // Params
  /*# Axes
    axis_linear_x: 3   # RS UD: Step length
    axis_linear_y: 2   # RS LR: Lateral fraction
    axis_linear_z: 1   # LS UD: Height
    axis_angular:  0   # LS LR: Yaw rate

    # D-pad (axes)
    leftright: 4       # D-pad LR: Step depth (discrete)
    updown: 5          # D-pad UD: Step height (discrete)

    # Buttons
    button_switch: 1   # A: switch stepping <-> RPY
    button_estop:  2   # X: E-STOP toggle

    # Bumpers
    lb: 8              # bottom left bumper: step velocity modulate
    rb: 9              # bottom right bumper: step velocity modulate
    lt: 6              # top left bumper: reset defaults
    rt: 7              # top right bumper: reset defaults
  */
  double frequency_{60.0};
  int axis_linear_x_{3}, axis_linear_y_{2}, axis_linear_z_{1}, axis_angular_{0};
  int button_switch_{1}, button_estop_{2}; 
  int rb_{5}, lb_{2}, rt_{5}, lt_{4}, updown_{5}, leftright_{4};
  double scale_linear_{1.0}, scale_angular_{1.0}, scale_bumper_{1.0};
  double debounce_thresh_{0.15};

  // Teleop core
  std::shared_ptr<tele::Teleop> teleop_;

  // ROS2 pub/sub/srv
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<spot_interfaces::msg::JoyButtons>::SharedPtr jb_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr switch_movement_client_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
