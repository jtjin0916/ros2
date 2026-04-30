#!/usr/bin/env python3
import copy
import os
import sys
import numpy as np

import rclpy
from rclpy.node import Node

# ROS2 msg imports (ROS2에서 mini_ros 메시지 패키지가 빌드되어 있어야 함)
from spot_interfaces.msg import MiniCmd, JoyButtons

# (선택) ament 경로
# from ament_index_python.packages import get_package_share_directory

# spotmicro imports (PYTHONPATH/패키징 정리가 필요)
####
from spot_gymenv.GymEnvs.spot_bezier_env import spotBezierEnv

####
from .Kinematics.SpotKinematics import SpotModel
from .GaitGenerator.Bezier import BezierGait

# Controller Params
STEPLENGTH_SCALE = 0.06
Z_SCALE_CTRL = 0.12
RPY_SCALE = 0.6
SV_SCALE = 0.1
CHPD_SCALE = 0.0005
YAW_SCALE = 1.5


class SpotCommander(Node):
    def __init__(self):
        super().__init__('policies')

        # 내부 상태
        self.mini_cmd = MiniCmd()
        self.jb = JoyButtons()

        # mini_cmd 초기화(ROS1 코드와 동일)
        self.mini_cmd.x_velocity = 0.0
        self.mini_cmd.y_velocity = 0.0
        self.mini_cmd.rate = 0.0
        self.mini_cmd.roll = 0.0
        self.mini_cmd.pitch = 0.0
        self.mini_cmd.yaw = 0.0
        self.mini_cmd.z = 0.0
        self.mini_cmd.motion = "Stop"
        self.mini_cmd.movement = "Stepping"

        # 파라미터화 (원하면 launch/yaml로 바꿀 수 있게)
        self.declare_parameter('control_hz', 600.0)
        self.declare_parameter('render', True)
        self.declare_parameter('on_rack', False)
        self.declare_parameter('height_field', False)
        self.declare_parameter('draw_foot_path', False)

        self.control_hz = float(self.get_parameter('control_hz').value)

        # 보행 파라미터 (원본 유지)
        self.BaseStepVelocity = 0.1
        self.StepVelocity = self.BaseStepVelocity

        self.BaseSwingPeriod = 0.2
        self.SwingPeriod = self.BaseSwingPeriod

        self.BaseClearanceHeight = 0.04
        self.BasePenetrationDepth = 0.005
        self.ClearanceHeight = self.BaseClearanceHeight
        self.PenetrationDepth = self.BasePenetrationDepth

        # 구독자
        self.sub_cmd = self.create_subscription(
            MiniCmd, 'mini_cmd', self.mini_cmd_cb, 10
        )
        self.sub_jb = self.create_subscription(
            JoyButtons, 'joybuttons', self.jb_cb, 10
        )

        # 시뮬/모델 로드
        self.load_spot()

        # 시간
        self.prev_time = self.get_clock().now()

        # 주기 실행(timer)
        period = 1.0 / self.control_hz
        self.timer = self.create_timer(period, self.move)

        self.get_logger().info("READY TO GO! (ROS2 Policies node)")

    def load_spot(self):
        render = bool(self.get_parameter('render').value)
        on_rack = bool(self.get_parameter('on_rack').value)
        height_field = bool(self.get_parameter('height_field').value)
        draw_foot_path = bool(self.get_parameter('draw_foot_path').value)

        self.env = spotBezierEnv(
            render=render,
            on_rack=on_rack,
            height_field=height_field,
            draw_foot_path=draw_foot_path
        )
        self.env.reset()

        seed = 0
        self.env.seed(seed)
        np.random.seed(seed)

        state_dim = self.env.observation_space.shape[0]
        action_dim = self.env.action_space.shape[0]
        max_action = float(self.env.action_space.high[0])

        self.get_logger().info(f"STATE DIM: {state_dim}")
        self.get_logger().info(f"ACTION DIM: {action_dim}")
        self.get_logger().info(f"RECORDED MAX ACTION: {max_action}")

        self.state = self.env.reset()

        self.spot = SpotModel()
        self.dt = self.env._time_step

        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)

        self.bzg = BezierGait(dt=self.env._time_step)

    def mini_cmd_cb(self, msg: MiniCmd):
        self.mini_cmd = msg

    def jb_cb(self, msg: JoyButtons):
        self.jb = msg

    def move(self):
        # dt 계산
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        # ---- 원본 move() 로직 유지 ----
        if self.mini_cmd.motion != "Stop":
            self.StepVelocity = self.BaseStepVelocity
            self.SwingPeriod = np.clip(
                self.BaseSwingPeriod +
                (-self.mini_cmd.faster + -self.mini_cmd.slower) * SV_SCALE,
                0.1, 0.3
            )

            if self.mini_cmd.movement == "Stepping":
                StepLength = self.mini_cmd.x_velocity + abs(self.mini_cmd.y_velocity * 0.66)
                StepLength = np.clip(StepLength, -1.0, 1.0)
                StepLength *= STEPLENGTH_SCALE

                LateralFraction = self.mini_cmd.y_velocity * np.pi / 2
                YawRate = self.mini_cmd.rate * YAW_SCALE

                pos = np.array([0.0, 0.0, self.mini_cmd.z * Z_SCALE_CTRL])
                orn = np.array([0.0, 0.0, 0.0])
            else:
                StepLength = 0.0
                LateralFraction = 0.0
                YawRate = 0.0

                self.ClearanceHeight = self.BaseClearanceHeight
                self.PenetrationDepth = self.BasePenetrationDepth
                self.StepVelocity = self.BaseStepVelocity

                pos = np.array([0.0, 0.0, self.mini_cmd.z * Z_SCALE_CTRL])
                orn = np.array([
                    self.mini_cmd.roll * RPY_SCALE,
                    self.mini_cmd.pitch * RPY_SCALE,
                    self.mini_cmd.yaw * RPY_SCALE
                ])
        else:
            StepLength = 0.0
            LateralFraction = 0.0
            YawRate = 0.0

            self.ClearanceHeight = self.BaseClearanceHeight
            self.PenetrationDepth = self.BasePenetrationDepth
            self.StepVelocity = self.BaseStepVelocity
            self.SwingPeriod = self.BaseSwingPeriod

            pos = np.array([0.0, 0.0, 0.0])
            orn = np.array([0.0, 0.0, 0.0])

        # clearance/penetration 조정
        self.ClearanceHeight += self.jb.updown * CHPD_SCALE
        self.PenetrationDepth += self.jb.leftright * CHPD_SCALE

        # 수동 reset
        if getattr(self.jb, 'left_bump', False) or getattr(self.jb, 'right_bump', False):
            self.ClearanceHeight = self.BaseClearanceHeight
            self.PenetrationDepth = self.BasePenetrationDepth
            self.StepVelocity = self.BaseStepVelocity
            self.SwingPeriod = self.BaseSwingPeriod
            self.env.reset()

        contacts = self.state[-4:]

        # swing 주기 반영
        self.bzg.Tswing = self.SwingPeriod

        self.T_bf = self.bzg.GenerateTrajectory(
            StepLength, LateralFraction, YawRate, self.StepVelocity,
            self.T_bf0, self.T_bf,
            self.ClearanceHeight, self.PenetrationDepth,
            contacts, dt
        )

        joint_angles = self.spot.IK(orn, pos, self.T_bf)
        self.env.pass_joint_angles(joint_angles.reshape(-1))

        # ---- 여기가 "강화학습(ARS) action"이 들어갈 자리 ----
        # 현재는 zero-action으로 step만 수행 (원본과 동일)
        action = np.zeros(self.env.action_space.shape[0], dtype=np.float32)

        #self.state, reward, done, _ = self.env.step(action)
        step_out = self.env.step(action)
        if len(step_out) == 5:
            self.state, reward, terminated, truncated, info = step_out
            done = bool(terminated or truncated)
        else:
            self.state, reward, done, info = step_out

        
        if done:
            self.env.reset()


def main():
    rclpy.init()
    node = SpotCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
