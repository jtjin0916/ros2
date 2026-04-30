#include "spot_cpp/spot.hpp"  // TODO/POINT: 헤더 경로를 qr_control 구조에 맞게 수정

#include <cmath>
#include <cstdio>
#include <iostream>

namespace spot
{
  // Spot Constructor
  Spot::Spot()
  {
    cmd.x_velocity = 0.0;
    cmd.y_velocity = 0.0;
    cmd.rate = 0.0;
    cmd.roll = 0.0;
    cmd.pitch = 0.0;
    cmd.yaw = 0.0;
    cmd.z = 0.0;
    cmd.faster = 0.0;
    cmd.slower = 0.0;
    cmd.motion = Stop;
    cmd.movement = Viewing;
  }

  void Spot::update_command(const double & vx, const double & vy, const double & z,
                            const double & w, const double & wx, const double & wy)
  {
    // If Command is nearly zero, just give zero
    if (almost_equal(vx, 0.0) && almost_equal(vy, 0.0) && almost_equal(z, 0.0) && almost_equal(w, 0.0))
    {
      cmd.motion = Stop;
      cmd.x_velocity = 0.0;
      cmd.y_velocity = 0.0;
      cmd.rate = 0.0;
      cmd.roll = 0.0;
      cmd.pitch = 0.0;
      cmd.yaw = 0.0;
      cmd.z = 0.0;
      cmd.faster = 0.0;
      cmd.slower = 0.0;
    }
    else
    {
      cmd.motion = Go;
      if (cmd.movement == Stepping)
      {
        // Stepping Mode, use commands as vx, vy, rate, Z
        cmd.x_velocity = vx;
        cmd.y_velocity = vy;
        cmd.rate = w;
        cmd.z = z;
        cmd.roll = 0.0;
        cmd.pitch = 0.0;
        cmd.yaw = 0.0;

        // change clearance height from +- 0-2 * scaling
        cmd.faster = 1.0 - wx;
        cmd.slower = -(1.0 - wy);
      }
      else
      {
        // Viewing Mode, use commands as RPY, Z
        cmd.x_velocity = 0.0;
        cmd.y_velocity = 0.0;
        cmd.rate = 0.0;
        cmd.roll = vy;
        cmd.pitch = vx;
        cmd.yaw = w;
        cmd.z = z;
        cmd.faster = 0.0;
        cmd.slower = 0.0;
      }
    }
  }

  void Spot::switch_movement()
  {
    // NOTE: original condition used "and" and checked NOT almost_equal for all 3,
    // meaning "only warn if all three are non-zero".
    // We keep same logic to match original behavior.
    if (!almost_equal(cmd.x_velocity, 0.0) && !almost_equal(cmd.y_velocity, 0.0) && !almost_equal(cmd.rate, 0.0))
    {
      // TODO/POINT: In ROS2, you probably want to log this from the Node using RCLCPP_WARN.
      std::cerr << "[spot] MAKE SURE BOTH LINEAR [" << cmd.x_velocity << ", " << cmd.y_velocity
                << "] AND ANGULAR VELOCITY [" << cmd.rate << "] ARE AT 0.0 BEFORE SWITCHING!\n";
      std::cerr << "[spot] STOPPING ROBOT...\n";

      cmd.motion = Stop;
      cmd.x_velocity = 0.0;
      cmd.y_velocity = 0.0;
      cmd.rate = 0.0;
      cmd.roll = 0.0;
      cmd.pitch = 0.0;
      cmd.yaw = 0.0;
      cmd.z = 0.0;
      cmd.faster = 0.0;
      cmd.slower = 0.0;
    }
    else
    {
      cmd.x_velocity = 0.0;
      cmd.y_velocity = 0.0;
      cmd.rate = 0.0;
      cmd.roll = 0.0;
      cmd.pitch = 0.0;
      cmd.yaw = 0.0;
      cmd.z = 0.0;
      cmd.faster = 0.0;
      cmd.slower = 0.0;

      if (cmd.movement == Viewing)
      {
        // TODO/POINT: log using node logger if you want
        std::cout << "[spot] SWITCHING TO STEPPING MOTION, COMMANDS NOW MAPPED TO VX|VY|W|Z.\n";
        cmd.movement = Stepping;
        cmd.motion = Stop;
      }
      else
      {
        std::cout << "[spot] SWITCHING TO VIEWING MOTION, COMMANDS NOW MAPPED TO R|P|Y|Z.\n";
        cmd.movement = Viewing;
        cmd.motion = Stop;
      }
    }
  }

  SpotCommand Spot::return_command() const
  {
    return cmd;
  }

}  // namespace spot
