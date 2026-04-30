#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import os
import time
import yaml
import numpy as np

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

# NOTE: busio/board/PCA9685는 DRY RUN일 때 없어도 되도록 "lazy import"로 처리한다.
# import busio, board
# from adafruit_pca9685 import PCA9685

from spot_interfaces.msg import JointAngles, JointPulse


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class PWMPublisher(Node):
    def __init__(self):
        super().__init__("pwm_publisher")

        # ---------- params ----------
        def p(name, default):
            self.declare_parameter(name, default)
            return self.get_parameter(name).value

        self.freq       = int(p("freq", 50))
        self.channels   = list(p("channels", list(range(12))))
        self.pulse_min  = int(p("pulse_min", 500))
        self.pulse_max  = int(p("pulse_max", 2500))

        addr_str = str(p("address", "0x40"))
        self.addr = int(addr_str, 16)

        self.topic   = str(p("topic", "/spot/joints"))
        self.dry_run = bool(p("dry_run", False))

        # calib yaml path (qr_control share/config)
        share_dir = get_package_share_directory("qr_control")
        default_calib = os.path.join(share_dir, "config", "servo_calib.yaml")
        self.calib_yaml = str(p("calib_yaml", default_calib))

        self.enable_pulse_override = bool(p("enable_pulse_override", True))
        self.angles_unit = str(p("angles_unit", "deg"))
        self.channel_to_joint = list(p("channel_to_joint", list(range(12))))
        self.auto_neutral_mid = bool(p("auto_neutral_mid", True))
        self.center_on_start  = bool(p("center_on_start", False))

        # ---------- defaults (원본 유지) ----------
        shoulder_lo, shoulder_hi = -0.548, 0.548
        elbow_lo,    elbow_hi    = -2.17,  2.17
        wrist_lo,    wrist_hi    = -2.17,  2.17
        self.lo  = [shoulder_lo, elbow_lo, wrist_lo] * 4
        self.hi  = [shoulder_hi, elbow_hi, wrist_hi] * 4
        self.off = [0.0, 0.1, 0.1, 0.0, 0.1, 0.1, 0.0, 0.1, 0.1, 0.0, 0.1, 0.1]
        self.dir = [ 1,  1,  1,  -1,  1, -1,   1,  1,  1,  -1,  1, -1]

        self.neutral_us  = None
        self.limit_lo_us = None
        self.limit_hi_us = None

        # ---------- load calib yaml ----------
        if self.calib_yaml and os.path.exists(self.calib_yaml):
            try:
                with open(self.calib_yaml, "r") as f:
                    y = yaml.safe_load(f) or {}

                if "offset" in y:
                    self.off = y["offset"]
                    if y.get("offset_unit", "rad") == "deg":
                        self.off = list(np.radians(self.off))

                if "direction" in y:
                    self.dir = y["direction"]

                if "limit_lo" in y:
                    self.lo = y["limit_lo"]

                if "limit_hi" in y:
                    self.hi = y["limit_hi"]

                if "neutral_us" in y:
                    self.neutral_us  = y["neutral_us"]
                    self.dir         = y.get("direction", self.dir)
                    self.limit_lo_us = y.get("limit_lo_us", [self.pulse_min] * 12)
                    self.limit_hi_us = y.get("limit_hi_us", [self.pulse_max] * 12)

                self.get_logger().info("Loaded calibration YAML.")
            except Exception as e:
                self.get_logger().warn(f"Failed to load calib_yaml: {e}")

        if self.neutral_us is None and self.auto_neutral_mid:
            mid = (self.pulse_min + self.pulse_max) // 2
            self.neutral_us  = [mid] * 12
            self.limit_lo_us = [self.pulse_min] * 12
            self.limit_hi_us = [self.pulse_max] * 12
            self.get_logger().info("No neutral_us in YAML → using mid as neutral for all joints.")

        # us_per_rad
        upr_default = p("us_per_rad", 636.0)
        if isinstance(upr_default, list):
            self.us_per_rad_pos = [float(x) for x in upr_default]
            self.us_per_rad_neg = [float(x) for x in upr_default]
        else:
            self.us_per_rad_pos = [float(upr_default)] * 12
            self.us_per_rad_neg = [float(upr_default)] * 12

        self.us_per_rad_pos = list(p("us_per_rad_pos", self.us_per_rad_pos))
        self.us_per_rad_neg = list(p("us_per_rad_neg", self.us_per_rad_neg))

        # effective rad limits based on microsecond bounds
        self.eff_lo_rad = [0.0] * 12
        self.eff_hi_rad = [0.0] * 12
        for i in range(12):
            mid   = float(self.neutral_us[i])
            lo_us = float(self.limit_lo_us[i])
            hi_us = float(self.limit_hi_us[i])

            upr_p = float(abs(self.us_per_rad_pos[i])) if float(self.us_per_rad_pos[i]) != 0 else 1e9
            upr_n = float(abs(self.us_per_rad_neg[i])) if float(self.us_per_rad_neg[i]) != 0 else 1e9

            self.eff_lo_rad[i] = (lo_us - mid) / upr_n  # negative
            self.eff_hi_rad[i] = (hi_us - mid) / upr_p  # positive

        # ---------- PCA9685 (lazy init) ----------
        self.pca = None
        if not self.dry_run:
            try:
                import busio, board
                from adafruit_pca9685 import PCA9685

                i2c = busio.I2C(board.SCL, board.SDA)
                self.pca = PCA9685(i2c, address=self.addr)
                self.pca.frequency = self.freq
                self.get_logger().info(f"PCA9685 ready @0x{self.addr:02X}, {self.freq} Hz")
            except Exception as e:
                self.get_logger().error(f"PCA9685 init failed: {e}")
                raise
        else:
            self.get_logger().warn("DRY RUN: skipping PCA9685 (busio/board) initialization.")

        if self.center_on_start and self.pca is not None:
            try:
                for ch in range(12):
                    self.write_us(ch, int(self.neutral_us[ch]))
            except Exception as e:
                self.get_logger().warn(f"[CALIB] center_on_start failed: {e}")

        # ---------- subs ----------
        self.create_subscription(JointAngles, self.topic, self.cb_joint, 1)
        self.get_logger().info(f"Subscribing: {self.topic} [JointAngles]")

        if self.enable_pulse_override:
            self.create_subscription(JointPulse, "/spot/pulse", self.cb_pulse, 1)
            self.get_logger().info("Calibration override enabled: /spot/pulse")

        self.get_logger().info(f"Effective dir: {self.dir}")
        self.get_logger().info(f"Effective us_per_rad_pos: {self.us_per_rad_pos}")
        self.get_logger().info(f"Effective us_per_rad_neg: {self.us_per_rad_neg}")
        self.get_logger().info("Eff. rad limits (lo..hi): " + str(
            [f"{self.eff_lo_rad[i]:.3f}..{self.eff_hi_rad[i]:.3f}" for i in range(12)]
        ))

        self._last_log_t = 0.0

    def write_us(self, ch, pulse_us):
        if self.pca is None:
            return
        period_us = int(1e6 / self.freq)
        duty = int(65535 * (float(pulse_us) / period_us))
        duty = clamp(duty, 0, 65535)
        self.pca.channels[int(ch)].duty_cycle = duty

    def cb_joint(self, msg: JointAngles):
        qraw = [
            msg.fls, msg.fle, msg.flw,
            msg.frs, msg.fre, msg.frw,
            msg.bls, msg.ble, msg.blw,
            msg.brs, msg.bre, msg.brw,
        ]
        if self.angles_unit.lower() == "deg":
            q = np.radians(qraw)
        else:
            q = np.array(qraw, dtype=float)

        last_ch = None
        last_us = None

        for ch in self.channels:
            ch = int(ch)
            if not (0 <= ch < 16):
                continue

            i = int(self.channel_to_joint[ch])
            v = self.dir[i] * (q[i] + self.off[i])

            v = clamp(v, self.eff_lo_rad[i], self.eff_hi_rad[i])

            mid   = float(self.neutral_us[i])
            upr_p = float(self.us_per_rad_pos[i])
            upr_n = float(self.us_per_rad_neg[i])

            if v >= 0.0:
                pulse_us = int(round(mid + upr_p * v))
            else:
                pulse_us = int(round(mid + upr_n * v))

            pulse_us = clamp(pulse_us, int(self.limit_lo_us[i]), int(self.limit_hi_us[i]))

            self.write_us(ch, pulse_us)
            last_ch = ch
            last_us = int(pulse_us)

        # throttle log ~0.5s
        t = time.time()
        if last_ch is not None and (t - self._last_log_t) >= 0.5:
            self._last_log_t = t
            self.get_logger().info(
                f"PWM {self.pulse_min}..{self.pulse_max}us, last_ch={last_ch}, last_us={last_us}"
            )

    def cb_pulse(self, jp: JointPulse):
        try:
            ch = int(jp.servo_num)
            us = int(jp.servo_pulse)
            if ch in [int(x) for x in self.channels] and 0 <= ch < 16:
                self.write_us(ch, us)
        except Exception as e:
            self.get_logger().warn(f"[CALIB] invalid JointPulse: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PWMPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
