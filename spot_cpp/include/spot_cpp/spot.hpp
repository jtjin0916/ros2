#ifndef SPOT_INCLUDE_GUARD_HPP
#define SPOT_INCLUDE_GUARD_HPP
/// \file
/// \brief Spots library which contains control functionality for Spot Mini Mini.

#include <cmath>    // std::fabs
#include <vector>   // (원본 유지, 실제로 안 쓰면 제거 가능)

namespace spot
{
    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    ///
    /// NOTE:
    /// - ROS1에서는 <ros/ros.h>가 우연히 fabs를 끌어왔을 수 있음.
    /// - ROS2/순수 C++로 가려면 <cmath> + std::fabs가 안전.
    constexpr bool almost_equal(double d1, double d2, double epsilon = 1.0e-1)
    {
        return (std::fabs(d1 - d2) < epsilon);
    }

    enum Motion { Go, Stop };
    enum Movement { Stepping, Viewing };

    /// \brief Struct to store the commanded type of motion, velocity and rate
    struct SpotCommand
    {
        Motion motion = Stop;
        Movement movement = Viewing;
        double x_velocity = 0.0;
        double y_velocity = 0.0;
        double rate = 0.0;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        double z = 0.0;
        double faster = 0.0;
        double slower = 0.0;
    };

    /// \brief Spot class responsible for high-level motion commands
    class Spot
    {
    public:
        /// \brief Constructor for Spot class
        Spot();

        /// \brief updates the type and velocity of motion to be commanded to the Spot
        /// \param vx: linear velocity (x)
        /// \param vy: linear velocity (y)
        /// \param z: robot height
        /// \param w: angular velocity
        /// \param wx: step height increase
        /// \param wy: step height decrease
        void update_command(const double & vx, const double & vy, const double & z,
                            const double & w, const double & wx, const double & wy);

        /// \brief changes the commanded motion from Forward/Backward to Left/Right or vice-versa
        void switch_movement();

        /// \brief returns the Spot's current command (Motion, v,w) for external use
        /// \returns SpotCommand (by value, same as original behavior)
        SpotCommand return_command() const;

    private:
        SpotCommand cmd;
    };

}  // namespace spot

#endif  // SPOT_INCLUDE_GUARD_HPP
