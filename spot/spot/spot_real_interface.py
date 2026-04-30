#!/usr/bin/env python3
"""
ROS2 port of mini_ros Policies node (QR project adaptation)

- Subscribes: mini_cmd, joybuttons, spot/imu, spot/contact
- Publishes: spot/agent, spot/joints

Goal of this edit:
- Keep ORIGINAL logic as-is.
- Only adapt imports / paths to QR project structure.
"""

from __future__ import annotations

import os
import copy
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ament_index_python.packages import get_package_share_directory

# ============================================================
# 1) MSG IMPORTS (spot project)
# ============================================================
# TODO/POINT: Replace package name with your actual msg package (qr_interfaces / qr_msgs / etc.)
from spot_interfaces.msg import MiniCmd, JoyButtons, IMUdata, ContactData, AgentData, JointAngles


# ============================================================
# 2) CORE LIB IMPORTS (QR project)
# ============================================================
# TODO/POINT: Replace these with your actual python package layout in QR project.
from Kinematics.SpotKinematics import SpotModel
from GaitGenerator.Bezier import BezierGait

# from qr_rl.ars_lib.ars import ARSAgent, Normalizer, Policy
##########강화학습 수정 중############
# --- RL (optional) ---
try:
    from spot_rl.ars_lib.ars import ARSAgent, Normalizer, Policy
    HAS_RL = True
except ImportError:
    HAS_RL = False
#######################################
# from qr_envs.spot_bezier_env import spotBezierEnv
# --- ENV (optional, only needed for loading ARS agent) ---
try:
    from spot_gymenv.GymEnvs.spot_bezier_env import spotBezierEnv
    HAS_ENVS = True
except ImportError:
    HAS_ENVS = False


class SpotCommander(Node):
    def __init__(self):
        super().__init__('policies')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # -------------------------
        # Parameters (keep same names as ROS1 if you want to reuse yaml)
        # -------------------------
        self.declare_parameter("STEPLENGTH_SCALE", 1.0)
        self.declare_parameter("Z_SCALE_CTRL", 1.0)
        self.declare_parameter("RPY_SCALE", 1.0)
        self.declare_parameter("SV_SCALE", 1.0)
        self.declare_parameter("CHPD_SCALE", 1.0)
        self.declare_parameter("YAW_SCALE", 1.0)

        self.declare_parameter("CD_SCALE", 1.0)
        self.declare_parameter("SLV_SCALE", 1.0)
        self.declare_parameter("RESIDUALS_SCALE", 1.0)
        self.declare_parameter("Z_SCALE", 1.0)
        self.declare_parameter("alpha", 0.0)
        self.declare_parameter("actions_to_filter", -1)

        self.declare_parameter("agent_num", 0)
        self.declare_parameter("BaseStepVelocity", 0.0)
        self.declare_parameter("Tswing", 0.25)
        self.declare_parameter("SwingPeriod_LIMITS", [0.15, 0.5])
        self.declare_parameter("BaseClearanceHeight", 0.04)
        self.declare_parameter("BasePenetrationDepth", 0.003)
        self.declare_parameter("ClearanceHeight_LIMITS", [0.01, 0.1])
        self.declare_parameter("PenetrationDepth_LIMITS", [0.0, 0.02])

        # spot model params
        self.declare_parameter("shoulder_length", 0.05)
        self.declare_parameter("elbow_length", 0.10)
        self.declare_parameter("wrist_length", 0.10)
        self.declare_parameter("hip_x", 0.10)
        self.declare_parameter("hip_y", 0.05)
        self.declare_parameter("foot_x", 0.00)
        self.declare_parameter("foot_y", 0.00)
        self.declare_parameter("height", 0.18)
        self.declare_parameter("com_offset", [0.0, 0.0, 0.0])

        self.declare_parameter("dt", 0.002)

        self.declare_parameter("Agent", True)
        self.declare_parameter("contacts", False)

        # ============================================================
        # 3) MODEL FILE PATH (QR project)
        # ============================================================
        # In ROS1 you used ../../spot_bullet/models/(contact|no_contact)
        # In ROS2 (QR project) best practice: put models under a package share directory.
        #
        # Example:
        #   qr_policy_models/
        #     share/qr_policy_models/
        #       contact/spot_ars_0_policy ...
        #       no_contact/spot_ars_0_policy ...
        #
        # TODO/POINT: Set this to the package name where you store your policy files.
        self.declare_parameter("policy_models_pkg", "qr_policy_models")
        self.declare_parameter("policy_models_contact_dir", "contact")
        self.declare_parameter("policy_models_nocontact_dir", "no_contact")
        self.declare_parameter("policy_file_prefix", "spot_ars_")

        # -------------------------
        # Read params
        # -------------------------
        self.STEPLENGTH_SCALE = float(self.get_parameter("STEPLENGTH_SCALE").value)
        self.Z_SCALE_CTRL = float(self.get_parameter("Z_SCALE_CTRL").value)
        self.RPY_SCALE = float(self.get_parameter("RPY_SCALE").value)
        self.SV_SCALE = float(self.get_parameter("SV_SCALE").value)
        self.CHPD_SCALE = float(self.get_parameter("CHPD_SCALE").value)
        self.YAW_SCALE = float(self.get_parameter("YAW_SCALE").value)

        self.CD_SCALE = float(self.get_parameter("CD_SCALE").value)
        self.RESIDUALS_SCALE = float(self.get_parameter("RESIDUALS_SCALE").value)
        self.Z_SCALE = float(self.get_parameter("Z_SCALE").value)

        self.alpha = float(self.get_parameter("alpha").value)
        self.actions_to_filter = int(self.get_parameter("actions_to_filter").value)

        self.Agent = bool(self.get_parameter("Agent").value)
        self.enable_contact = bool(self.get_parameter("contacts").value)

        # Agent 사용 가능 조건: RL + ENVS 둘 다 있어야 함############################################수정용 강제 오프 처리
        if self.Agent and not (HAS_RL and HAS_ENVS):
            self.get_logger().warn(
                f"Agent requested but dependencies missing: HAS_RL={HAS_RL}, HAS_ENVS={HAS_ENVS}. "
                "Forcing Agent=False (Bezier+IK only)."
            )
            self.Agent = False
        #####################################################################################

        self.agent_num = int(self.get_parameter("agent_num").value)

        self.BaseStepVelocity = float(self.get_parameter("BaseStepVelocity").value)
        self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)

        self.BaseSwingPeriod = float(self.get_parameter("Tswing").value)
        self.SwingPeriod = copy.deepcopy(self.BaseSwingPeriod)
        self.SwingPeriod_LIMITS = list(self.get_parameter("SwingPeriod_LIMITS").value)

        self.BaseClearanceHeight = float(self.get_parameter("BaseClearanceHeight").value)
        self.BasePenetrationDepth = float(self.get_parameter("BasePenetrationDepth").value)
        self.ClearanceHeight = copy.deepcopy(self.BaseClearanceHeight)
        self.PenetrationDepth = copy.deepcopy(self.BasePenetrationDepth)
        self.ClearanceHeight_LIMITS = list(self.get_parameter("ClearanceHeight_LIMITS").value)
        self.PenetrationDepth_LIMITS = list(self.get_parameter("PenetrationDepth_LIMITS").value)

        self.policy_models_pkg = str(self.get_parameter("policy_models_pkg").value)
        self.policy_models_contact_dir = str(self.get_parameter("policy_models_contact_dir").value)
        self.policy_models_nocontact_dir = str(self.get_parameter("policy_models_nocontact_dir").value)
        self.policy_file_prefix = str(self.get_parameter("policy_file_prefix").value)

        # -------------------------
        # State holders
        # -------------------------
        self.movetypes = ["Stop"]
        self.mini_cmd = MiniCmd()
        self.jb = JoyButtons()

        self.mini_cmd.x_velocity = 0.0
        self.mini_cmd.y_velocity = 0.0
        self.mini_cmd.rate = 0.0
        self.mini_cmd.roll = 0.0
        self.mini_cmd.pitch = 0.0
        self.mini_cmd.yaw = 0.0
        self.mini_cmd.z = 0.0
        self.mini_cmd.motion = "Stop"
        self.mini_cmd.movement = "Stepping"

        self.contacts = [0, 0, 0, 0]
        self.imu = [0.0] * 8

        # -------------------------
        # Model objects
        # -------------------------
        self.spot = SpotModel(
            shoulder_length=float(self.get_parameter("shoulder_length").value),
            elbow_length=float(self.get_parameter("elbow_length").value),
            wrist_length=float(self.get_parameter("wrist_length").value),
            hip_x=float(self.get_parameter("hip_x").value),
            hip_y=float(self.get_parameter("hip_y").value),
            foot_x=float(self.get_parameter("foot_x").value),
            foot_y=float(self.get_parameter("foot_y").value),
            height=float(self.get_parameter("height").value),
            com_offset=list(self.get_parameter("com_offset").value),
        )

        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)

        self.bzg = BezierGait(
            dt=float(self.get_parameter("dt").value),
            Tswing=float(self.get_parameter("Tswing").value),
        )

        # Agent
        if self.Agent:
            self.load_spot(contacts=self.enable_contact, agent_num=self.agent_num)

        # -------------------------
        # ROS2 pubs/subs
        # -------------------------
        self.sub_cmd = self.create_subscription(MiniCmd, 'mini_cmd', self.cmd_cb, qos)
        self.sub_jb = self.create_subscription(JoyButtons, 'joybuttons', self.jb_cb, qos)
        self.sub_imu = self.create_subscription(IMUdata, 'spot/imu', self.imu_cb, qos)
        self.sub_cnt = self.create_subscription(ContactData, 'spot/contact', self.cnt_cb, qos)

        self.ag_pub = self.create_publisher(AgentData, 'spot/agent', qos)
        self.ja_pub = self.create_publisher(JointAngles, 'spot/joints', qos)

        # Timing
        self.last_time_ns = self.get_clock().now().nanoseconds
        self.control_hz = 600.0
        self.timer = self.create_timer(1.0 / self.control_hz, self.move)

        self.get_logger().info("READY TO GO! (ROS2, QR-adapted imports/paths)")

    def load_spot(self, contacts: bool, state_dim=12, action_dim=14, agent_num=0):
        """
        Keep original behavior: Policy/Normalizer + ARSAgent.load()

        TODO/POINT:
        - If contacts=True, your policy might have been trained with state_dim=16.
          In original code, it still constructed state with contacts when enabled.
          Ensure your model actually matches this.
        """

        ###########수정중###############
        if not (HAS_RL and HAS_ENVS):
            self.get_logger().warn("load_spot() skipped: RL/ENV not available.")
            return
        #########   ######## ##################

        self.policy = Policy(state_dim=state_dim, action_dim=action_dim)
        self.normalizer = Normalizer(state_dim=state_dim)

        env = spotBezierEnv(render=False, on_rack=False, height_field=False, draw_foot_path=False)
        agent = ARSAgent(self.normalizer, self.policy, env)

        # --------- QR project model path resolution ----------
        # Use package share dir (ROS2 recommended)
        # TODO/POINT: make sure qr_policy_models is installed and contains contact/no_contact folders.
        models_share = get_package_share_directory(self.policy_models_pkg)
        subdir = self.policy_models_contact_dir if contacts else self.policy_models_nocontact_dir
        models_path = os.path.join(models_share, subdir)

        file_name = self.policy_file_prefix  # default: "spot_ars_"
        policy_path = os.path.join(models_path, f"{file_name}{agent_num}_policy")

        self.get_logger().info(f"MODEL PATH: {models_path}")

        if os.path.exists(policy_path):
            self.get_logger().info(f"Loading Existing agent: {agent_num}")
            agent.load(os.path.join(models_path, f"{file_name}{agent_num}"))
            agent.policy.episode_steps = np.inf
            self.policy = agent.policy
        else:
            self.get_logger().warn(f"Policy not found at: {policy_path}")

        self.action = np.zeros(action_dim, dtype=np.float32)
        self.old_act = self.action[:self.actions_to_filter] if self.actions_to_filter >= 0 else self.action.copy()

    # Callbacks
    def imu_cb(self, imu: IMUdata):
        try:
            # TODO/POINT: confirm IMUdata fields/units in QR project match these names.
            self.imu = [
                imu.roll, imu.pitch,
                np.radians(imu.gyro_x),
                np.radians(imu.gyro_y),
                np.radians(imu.gyro_z),
                imu.acc_x, imu.acc_y, imu.acc_z - 9.81
            ]
        except Exception as e:
            self.get_logger().warn(f"imu_cb exception: {e}")

    def cnt_cb(self, cnt: ContactData):
        try:
            # TODO/POINT: confirm ContactData field names in QR project.
            self.contacts = [cnt.FL, cnt.FR, cnt.BL, cnt.BR]
        except Exception as e:
            self.get_logger().warn(f"cnt_cb exception: {e}")

    def cmd_cb(self, mini_cmd: MiniCmd):
        self.mini_cmd = mini_cmd

    def jb_cb(self, jb: JoyButtons):
        self.jb = jb

    def move(self):
        # NOTE: keep original logic as close as possible.
        now_ns = self.get_clock().now().nanoseconds
        dt = (now_ns - self.last_time_ns) * 1e-9
        if dt <= 0.0:
            dt = 1.0 / self.control_hz
        self.last_time_ns = now_ns

        # Move Type
        if self.mini_cmd.movement == "Stepping":
            step_or_view = False
        else:
            step_or_view = True

        if self.mini_cmd.motion != "Stop":
            self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)
            self.SwingPeriod = np.clip(
                copy.deepcopy(self.BaseSwingPeriod) +
                (-self.mini_cmd.faster + -self.mini_cmd.slower) * self.SV_SCALE,
                self.SwingPeriod_LIMITS[0], self.SwingPeriod_LIMITS[1])

            if self.mini_cmd.movement == "Stepping":
                StepLength = self.mini_cmd.x_velocity + abs(self.mini_cmd.y_velocity * 0.66)
                StepLength = np.clip(StepLength, -1.0, 1.0)
                StepLength *= self.STEPLENGTH_SCALE
                LateralFraction = self.mini_cmd.y_velocity * np.pi / 2
                YawRate = self.mini_cmd.rate * self.YAW_SCALE
                pos = np.array([0.0, 0.0, 0.0])
                orn = np.array([0.0, 0.0, 0.0])
            else:
                StepLength = 0.0
                LateralFraction = 0.0
                YawRate = 0.0
                self.ClearanceHeight = copy.deepcopy(self.BaseClearanceHeight)
                self.PenetrationDepth = copy.deepcopy(self.BasePenetrationDepth)
                self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)
                pos = np.array([0.0, 0.0, self.mini_cmd.z * self.Z_SCALE_CTRL])
                orn = np.array([
                    self.mini_cmd.roll * self.RPY_SCALE,
                    self.mini_cmd.pitch * self.RPY_SCALE,
                    self.mini_cmd.yaw * self.RPY_SCALE
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

        # TODO: integrate into controller
        # TODO/POINT: confirm JoyButtons fields exist: updown, leftright, left_bump, right_bump
        self.ClearanceHeight += self.jb.updown * self.CHPD_SCALE
        self.PenetrationDepth += self.jb.leftright * self.CHPD_SCALE

        if self.jb.left_bump or self.jb.right_bump:
            self.ClearanceHeight = copy.deepcopy(self.BaseClearanceHeight)
            self.PenetrationDepth = copy.deepcopy(self.BasePenetrationDepth)
            self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)
            self.SwingPeriod = copy.deepcopy(self.BaseSwingPeriod)

        # OPTIONAL: Agent
        if self.Agent and self.mini_cmd.motion != "Stop":
            phases = copy.deepcopy(self.bzg.Phases)
            state = []
            state.extend(self.imu)
            state.extend(phases)
            if self.enable_contact:
                state.extend(self.contacts)

            self.normalizer.observe(state)
            # Don't normalize contacts
            # TODO/POINT: if enable_contact=True, original assumes last 4 are contacts.
            if self.enable_contact:
                state[:-4] = self.normalizer.normalize(state)[:-4]
            else:
                state = self.normalizer.normalize(state)

            self.action = self.policy.evaluate(state, None, None)
            self.action = np.tanh(self.action)

            # EXP FILTER
            # TODO/POINT: ensure actions_to_filter matches your intention (-1 means all)
            self.action[:self.actions_to_filter] = self.alpha * self.old_act + (
                1.0 - self.alpha) * self.action[:self.actions_to_filter]
            self.old_act = self.action[:self.actions_to_filter]

            self.ClearanceHeight += self.action[0] * self.CD_SCALE

        # Update Step Period
        self.bzg.Tswing = self.SwingPeriod

        # CLIP
        self.ClearanceHeight = np.clip(self.ClearanceHeight,
                                       self.ClearanceHeight_LIMITS[0],
                                       self.ClearanceHeight_LIMITS[1])
        self.PenetrationDepth = np.clip(self.PenetrationDepth,
                                        self.PenetrationDepth_LIMITS[0],
                                        self.PenetrationDepth_LIMITS[1])

        self.T_bf = self.bzg.GenerateTrajectory(
            StepLength, LateralFraction, YawRate, self.StepVelocity,
            self.T_bf0, self.T_bf, self.ClearanceHeight, self.PenetrationDepth,
            self.contacts, dt
        )

        T_bf_copy = copy.deepcopy(self.T_bf)

        if self.Agent and self.mini_cmd.motion != "Stop":
            self.action[2:] *= self.RESIDUALS_SCALE
            # TODO/POINT: confirm T_bf dict keys match ("FL","FR","BL","BR") in your QR BezierGait.
            T_bf_copy["FL"][:3, 3] += self.action[2:5]
            T_bf_copy["FR"][:3, 3] += self.action[5:8]
            T_bf_copy["BL"][:3, 3] += self.action[8:11]
            T_bf_copy["BR"][:3, 3] += self.action[11:14]
            pos[2] += abs(self.action[1]) * self.Z_SCALE

        joint_angles = self.spot.IK(orn, pos, T_bf_copy)

        ja_msg = JointAngles()
        ja_msg.fls = np.degrees(joint_angles[0][0])
        ja_msg.fle = np.degrees(joint_angles[0][1])
        ja_msg.flw = np.degrees(joint_angles[0][2])

        ja_msg.frs = np.degrees(joint_angles[1][0])
        ja_msg.fre = np.degrees(joint_angles[1][1])
        ja_msg.frw = np.degrees(joint_angles[1][2])

        ja_msg.bls = np.degrees(joint_angles[2][0])
        ja_msg.ble = np.degrees(joint_angles[2][1])
        ja_msg.blw = np.degrees(joint_angles[2][2])

        ja_msg.brs = np.degrees(joint_angles[3][0])
        ja_msg.bre = np.degrees(joint_angles[3][1])
        ja_msg.brw = np.degrees(joint_angles[3][2])

        ja_msg.step_or_view = step_or_view
        self.ja_pub.publish(ja_msg)


def main():
    rclpy.init()
    node = SpotCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
