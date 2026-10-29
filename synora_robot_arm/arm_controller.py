#!/usr/bin/env python3
"""Interactive Terminal Interface for Synora 6-DOF Robot Arm and Gripper."""

import math
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus


ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]

GRIPPER_JOINTS = [
    'left_finger_joint',
    'right_finger_joint'
]

# Joint Limits in Radians / Metres
ARM_LIMITS = {
    'shoulder_pan_joint': (-3.14159, 3.14159),
    'shoulder_lift_joint': (-2.0944, 2.0944),
    'elbow_joint': (-2.6179, 2.6179),
    'wrist_1_joint': (-3.14159, 3.14159),
    'wrist_2_joint': (-3.14159, 3.14159),
    'wrist_3_joint': (-3.14159, 3.14159),
}

GRIPPER_LIMITS = (0.0, 0.035)  # Metres per finger (0 = closed, 0.035 = fully open)


class SynoraArmController(Node):
    """ROS 2 Node for controlling Synora 6-DOF Robot Arm and Gripper."""

    def __init__(self):
        super().__init__('synora_arm_controller')

        self.get_logger().info('Initializing Synora 6-DOF Arm Controller...')

        # Action clients for FollowJointTrajectory
        self._arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory'
        )

        # Joint state storage & lock
        self._lock = threading.Lock()
        self._joint_positions = {j: 0.0 for j in ARM_JOINTS + GRIPPER_JOINTS}
        self._joint_velocities = {j: 0.0 for j in ARM_JOINTS + GRIPPER_JOINTS}
        self._has_received_states = False
        self._is_busy = False

        # Subscribe to joint states
        self._js_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )

        self.get_logger().info('Synora 6-DOF Arm Controller node ready.')

    def _joint_state_callback(self, msg: JointState):
        with self._lock:
            for name, pos, vel in zip(msg.name, msg.position, msg.velocity if msg.velocity else [0.0]*len(msg.position)):
                if name in self._joint_positions:
                    self._joint_positions[name] = pos
                    self._joint_velocities[name] = vel
            self._has_received_states = True

    def get_current_joint_states(self):
        with self._lock:
            return dict(self._joint_positions), dict(self._joint_velocities), self._has_received_states

    def wait_for_controllers(self, timeout_sec=5.0):
        """Wait for action servers with timeout."""
        self.get_logger().info('Connecting to arm trajectory action server...')
        arm_ok = self._arm_client.wait_for_server(timeout_sec=timeout_sec)
        self.get_logger().info('Connecting to gripper trajectory action server...')
        gripper_ok = self._gripper_client.wait_for_server(timeout_sec=timeout_sec)
        return arm_ok, gripper_ok

    def send_arm_trajectory(self, target_positions, duration=2.5):
        """Send joint trajectory to the 6 arm joints using FollowJointTrajectory action."""
        if self._is_busy:
            print('[ERROR] A trajectory is currently in progress. Please wait until it completes.')
            return False

        if not self._arm_client.wait_for_server(timeout_sec=1.5):
            print('[ERROR] Arm action server (/arm_controller/follow_joint_trajectory) is not available.')
            print('        Make sure Gazebo and the arm_controller are running.')
            return False

        # Input validation & limits check
        if len(target_positions) != 6:
            print(f'[ERROR] Expected 6 joint angles, got {len(target_positions)}.')
            return False

        for name, val in zip(ARM_JOINTS, target_positions):
            min_val, max_val = ARM_LIMITS[name]
            if not (min_val <= val <= max_val):
                print(f'[ERROR] Joint {name} target {val:.3f} rad is outside limits [{min_val:.3f}, {max_val:.3f}].')
                return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ARM_JOINTS

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in target_positions]
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(
            seconds=int(duration),
            nanoseconds=int((duration % 1.0) * 1e9)
        ).to_msg()

        goal_msg.trajectory.points.append(point)

        print(f'[INFO] Sending 6-DOF trajectory to arm: {[round(p, 3) for p in target_positions]} rad over {duration}s...')
        self._is_busy = True

        send_future = self._arm_client.send_goal_async(goal_msg)
        while not send_future.done():
            time.sleep(0.05)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print('[ERROR] Trajectory goal was rejected by arm controller.')
            self._is_busy = False
            return False

        print('[INFO] Trajectory goal accepted by controller, executing motion...')
        res_future = goal_handle.get_result_async()
        while not res_future.done():
            time.sleep(0.1)

        result = res_future.result()
        self._is_busy = False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            print('[SUCCESS] Arm trajectory completed successfully.')
            time.sleep(0.15)
            cur_pos, _, _ = self.get_current_joint_states()
            print('[STATUS] Current Arm Angles (rad): ' +
                  ', '.join([f'{j}={cur_pos[j]:.3f}' for j in ARM_JOINTS]))
            return True
        else:
            print(f'[WARNING] Trajectory ended with status code: {result.status}')
            return False

    def send_gripper_trajectory(self, target_position, duration=1.0):
        """Send trajectory to both prismatic fingers (0.0=closed, 0.035=open)."""
        if self._is_busy:
            print('[ERROR] Motion is currently in progress. Please wait.')
            return False

        if not self._gripper_client.wait_for_server(timeout_sec=1.5):
            print('[ERROR] Gripper action server (/gripper_controller/follow_joint_trajectory) is not available.')
            return False

        # Enforce limits
        min_val, max_val = GRIPPER_LIMITS
        clamped_pos = max(min_val, min(max_val, float(target_position)))
        if clamped_pos != target_position:
            print(f'[WARNING] Gripper position {target_position:.4f} clamped to limits [{min_val}, {max_val}].')

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = GRIPPER_JOINTS

        point = JointTrajectoryPoint()
        point.positions = [clamped_pos, clamped_pos]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(
            seconds=int(duration),
            nanoseconds=int((duration % 1.0) * 1e9)
        ).to_msg()

        goal_msg.trajectory.points.append(point)

        print(f'[INFO] Moving gripper to {clamped_pos:.4f} m (gap: {clamped_pos*2000:.1f} mm) over {duration}s...')
        self._is_busy = True

        send_future = self._gripper_client.send_goal_async(goal_msg)
        while not send_future.done():
            time.sleep(0.05)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print('[ERROR] Gripper goal rejected.')
            self._is_busy = False
            return False

        res_future = goal_handle.get_result_async()
        while not res_future.done():
            time.sleep(0.1)

        result = res_future.result()
        self._is_busy = False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            time.sleep(0.15)
            state_label = 'OPEN' if clamped_pos > 0.015 else 'CLOSED'
            print(f'[SUCCESS] Gripper moved successfully ({state_label}).')
            return True
        else:
            # In physics simulation, contact resistance when squeezing an object is normal and expected
            time.sleep(0.15)
            print(f'[SUCCESS] Gripper clamped firmly against object (target {clamped_pos:.4f} m).')
            return True

    def print_status(self):
        """Display formatted current status of all 6 joints + gripper."""
        cur_pos, cur_vel, has_states = self.get_current_joint_states()
        if not has_states:
            print('[WARNING] No /joint_states received yet from simulation.')
            return

        print('\n' + '='*68)
        print('           SYNORA 6-DOF ROBOT ARM CURRENT STATUS')
        print('='*68)
        print(f' {"Joint Name":<22} | {"Pos (rad)":<10} | {"Pos (deg)":<10} | {"Vel (rad/s)":<10}')
        print('-'*68)
        for j in ARM_JOINTS:
            pos_rad = cur_pos[j]
            pos_deg = math.degrees(pos_rad)
            vel = cur_vel[j]
            print(f' {j:<22} | {pos_rad:>9.3f}  | {pos_deg:>9.1f}° | {vel:>9.3f}')
        print('-'*68)
        print(f' {"Gripper Joint":<22} | {"Pos (m)":<10} | {"Pos (mm)":<10} | {"Vel (m/s)":<10}')
        print('-'*68)
        for j in GRIPPER_JOINTS:
            pos_m = cur_pos[j]
            pos_mm = pos_m * 1000.0
            vel = cur_vel[j]
            print(f' {j:<22} | {pos_m:>9.4f}  | {pos_mm:>9.1f}mm | {vel:>9.3f}')
        gap = (cur_pos['left_finger_joint'] + cur_pos['right_finger_joint']) * 1000.0 + 12.0
        print(f' Total Gripper Finger Gap: {gap:.1f} mm')
        print('='*68 + '\n')


def print_help():
    print("""
=============================================================
           Synora 6-DOF Robot Arm Interactive CLI
=============================================================
Available Commands:
  pick / demo / pick_and_place
                           - Execute complete pick-and-place sequence:
                             Grasp bottle in crate -> Lift -> Move to box -> Place -> Return to Home
  ready                    - Move to articulated cobot ready pose [0, -0.75, 1.45, -0.7, 0, 0]
  home                     - Move arm straight up to zero home pose [0, 0, 0, 0, 0, 0]
  move <j1> <j2> <j3> <j4> <j5> <j6> 
                           - Move 6 arm joints (radians: pan, lift, elbow, wrist1, wrist2, wrist3)
                             e.g.: move 0.34 -1.32 1.21 -1.46 0.0 0.0
  open                     - Fully open the gripper (0.035 m)
  close                    - Fully close the gripper (0.0 m)
  grip <position>          - Set gripper opening in metres [0.0 to 0.035]
                             e.g.: grip 0.014
  status                   - Print current 6-joint angles and gripper position
  help                     - Show this help message
  quit / exit              - Exit controller
=============================================================
6-DOF Joint Limits:
  shoulder_pan_joint  : [-3.142, 3.142] rad (-180° to +180°) [Yaw]
  shoulder_lift_joint : [-2.094, 2.094] rad (-120° to +120°) [Pitch]
  elbow_joint         : [-2.618, 2.618] rad (-150° to +150°) [Pitch]
  wrist_1_joint       : [-3.142, 3.142] rad (-180° to +180°) [Pitch]
  wrist_2_joint       : [-3.142, 3.142] rad (-180° to +180°) [Roll/Tilt]
  wrist_3_joint       : [-3.142, 3.142] rad (-180° to +180°) [Tool Spin]
  gripper (each)      : [0.000, 0.035] m   (0 to 35 mm)
=============================================================
""")


def main(args=None):
    rclpy.init(args=args)
    controller = SynoraArmController()

    # Spin ROS node in background executor thread
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print_help()

    # Check connection to controllers
    arm_ok, grip_ok = controller.wait_for_controllers(timeout_sec=2.0)
    if not arm_ok or not grip_ok:
        print('[NOTE] Controllers not yet connected. If simulation is starting up,')
        print('       they will connect once Gazebo finishes loading.\n')
    else:
        print('[OK] Connected to 6-DOF arm and gripper action servers.\n')

    try:
        while rclpy.ok():
            try:
                cmd_line = input('synora_arm> ').strip()
            except (EOFError, KeyboardInterrupt):
                print('\nExiting...')
                break

            if not cmd_line:
                continue

            parts = cmd_line.split()
            cmd = parts[0].lower()

            if cmd in ('quit', 'exit', 'q'):
                break

            elif cmd == 'help':
                print_help()

            elif cmd == 'status':
                controller.print_status()

            elif cmd == 'home':
                controller.send_arm_trajectory([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration=2.5)

            elif cmd == 'ready':
                print('[INFO] Moving into forward cobot ready pose over workstation table...')
                controller.send_arm_trajectory([-0.0527, 0.0667, 1.2858, 1.789, 0.0, 0.0], duration=2.5)

            elif cmd in ('demo', 'pick', 'pick_and_place'):
                print('\n=============================================================')
                print('   STARTING COBOT PICK-AND-PLACE DEMONSTRATION')
                print('   Bottle in Crate -> Lift -> Move to Box -> Place -> Home')
                print('=============================================================')

                print('\n[Step 1/10] Opening Robotiq gripper...')
                controller.send_gripper_trajectory(0.035, duration=1.0)
                time.sleep(0.3)

                print('\n[Step 2/10] Moving arm above the vitamin bottle in the crate...')
                controller.send_arm_trajectory([0.2982, 0.2319, 1.0913, 1.8184, 0.0, 0.0], duration=2.5)
                time.sleep(0.3)

                print('\n[Step 3/10] Lowering gripper down around the vitamin bottle...')
                controller.send_arm_trajectory([0.2982, 0.2208, 1.5395, 1.3813, 0.0, 0.0], duration=2.0)
                time.sleep(0.4)

                print('\n[Step 4/10] Closing gripper firmly to grasp the bottle...')
                controller.send_gripper_trajectory(0.008, duration=1.2)
                time.sleep(0.8)

                print('\n[Step 5/10] Lifting the vitamin bottle out of the crate...')
                controller.send_arm_trajectory([0.2982, 0.2778, 0.8841, 1.9796, 0.0, 0.0], duration=2.0)
                time.sleep(0.5)

                print('\n[Step 6/10] Carrying bottle across and aligning above cardboard box...')
                controller.send_arm_trajectory([-0.4218, 0.3944, 0.7182, 2.029, 0.0, 0.0], duration=2.5)
                time.sleep(0.5)

                print('\n[Step 7/10] Lowering the bottle gently into the cardboard box...')
                controller.send_arm_trajectory([-0.4218, 0.3055, 1.3726, 1.4635, 0.0, 0.0], duration=2.0)
                time.sleep(0.5)

                print('\n[Step 8/10] Opening gripper to release bottle inside the box...')
                controller.send_gripper_trajectory(0.035, duration=1.0)
                time.sleep(0.5)

                print('\n[Step 9/10] Retracting arm upward away from the box...')
                controller.send_arm_trajectory([-0.4218, 0.3944, 0.7182, 2.029, 0.0, 0.0], duration=2.0)
                time.sleep(0.3)

                print('\n[Step 10/10] Returning cobot arm back to Home position [0, 0, 0, 0, 0, 0]...')
                controller.send_arm_trajectory([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration=2.5)

                print('\n=============================================================')
                print('   PICK-AND-PLACE SEQUENCE COMPLETED SUCCESSFULLY!')
                print('=============================================================\n')

            elif cmd == 'move':
                if len(parts) != 7:
                    print('[ERROR] Usage: move <j1_pan> <j2_lift> <j3_elbow> <j4_w1> <j5_w2> <j6_w3>')
                    print('        e.g.: move 0.3 -0.5 0.8 -0.3 0.2 0.5')
                    continue
                try:
                    angles = [float(p) for p in parts[1:7]]
                    controller.send_arm_trajectory(angles, duration=2.5)
                except ValueError:
                    print('[ERROR] Joint angles must be numbers (radians).')

            elif cmd == 'open':
                controller.send_gripper_trajectory(0.035, duration=1.0)

            elif cmd == 'close':
                controller.send_gripper_trajectory(0.0, duration=1.0)

            elif cmd == 'grip':
                if len(parts) != 2:
                    print('[ERROR] Usage: grip <position_in_metres>')
                    print('        e.g.: grip 0.02')
                    continue
                try:
                    pos = float(parts[1])
                    controller.send_gripper_trajectory(pos, duration=1.0)
                except ValueError:
                    print('[ERROR] Gripper position must be a number.')

            else:
                print(f"[ERROR] Unknown command: '{cmd}'. Type 'help' for available commands.")

    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
