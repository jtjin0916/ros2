#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from spot_interfaces.msg import IMUdata

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ACCELEROMETER,
)


def quaternion_to_euler_rad(x: float, y: float, z: float, w: float):
    """Convert quaternion to Euler angles in radians."""
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class BNO085Publisher(Node):
    def __init__(self):
        super().__init__("bno085_publisher")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.declare_parameter("topic_name", "spot/imu")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("debug_log", True)

        self.topic_name = str(self.get_parameter("topic_name").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.debug_log = bool(self.get_parameter("debug_log").value)

        if self.publish_hz <= 0.0:
            self.publish_hz = 50.0

        self.pub = self.create_publisher(IMUdata, self.topic_name, qos)

        self._init_sensor()

        self.last_debug_time = time.time()
        self.timer = self.create_timer(1.0 / self.publish_hz, self.publish_imu)

        self.get_logger().info(
            f"BNO085 publisher started. topic={self.topic_name}, hz={self.publish_hz}"
        )

    def _init_sensor(self):
        """Initialize BNO085 over I2C."""
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(i2c)

            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)

            self.get_logger().info("BNO085 initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize BNO085: {e}")
            raise

    def publish_imu(self):
        try:
            msg = IMUdata()

            # BNO085 quaternion order is typically (x, y, z, w)
            quat = self.bno.quaternion
            if quat is None or len(quat) < 4:
                self.get_logger().warn("Quaternion data unavailable.")
                return

            qx, qy, qz, qw = quat[0], quat[1], quat[2], quat[3]
            roll_rad, pitch_rad, yaw_rad = quaternion_to_euler_rad(qx, qy, qz, qw)

            # qr_motion_control_node expects roll/pitch as angle values.
            # To match the original style safely, publish them in degrees.
            msg.roll = float(math.degrees(roll_rad))
            msg.pitch = float(math.degrees(pitch_rad))
            msg.yaw = float(math.degrees(yaw_rad))

            # BNO085 gyro is usually rad/s. Convert to deg/s because
            # qr_motion_control_node does np.radians(...) internally.
            gyro = self.bno.gyro
            if gyro is None or len(gyro) < 3:
                gx_deg = gy_deg = gz_deg = 0.0
            else:
                gx_deg = math.degrees(float(gyro[0]))
                gy_deg = math.degrees(float(gyro[1]))
                gz_deg = math.degrees(float(gyro[2]))

            msg.gyro_x = gx_deg
            msg.gyro_y = gy_deg
            msg.gyro_z = gz_deg

            # BNO085 acceleration is usually m/s^2.
            # qr_motion_control_node subtracts 9.81 from acc_z internally.
            acc = self.bno.acceleration
            if acc is None or len(acc) < 3:
                ax = ay = az = 0.0
            else:
                ax = float(acc[0])
                ay = float(acc[1])
                az = float(acc[2])

            msg.acc_x = ax
            msg.acc_y = ay
            msg.acc_z = az

            self.pub.publish(msg)

            if self.debug_log and (time.time() - self.last_debug_time) > 1.0:
                self.last_debug_time = time.time()
                self.get_logger().info(
                    "IMU "
                    f"rpy_deg=({msg.roll:.2f}, {msg.pitch:.2f}, {msg.yaw:.2f}) "
                    f"gyro_deg_s=({msg.gyro_x:.2f}, {msg.gyro_y:.2f}, {msg.gyro_z:.2f}) "
                    f"acc=({msg.acc_x:.2f}, {msg.acc_y:.2f}, {msg.acc_z:.2f})"
                )

        except Exception as e:
            self.get_logger().warn(f"Failed to read/publish BNO085 data: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BNO085Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
