#!/usr/bin/env python3
"""Dedicated Gripper Controller node for Synora Robot Arm."""

import argparse
import sys
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus


class SynoraGripperController(Node):
    """Controls the two-finger gripper via FollowJointTrajectory action."""

    def __init__(self):
        super().__init__('synora_gripper_controller')
        self._client = ActionClient(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory'
        )
        self.joint_names = ['left_finger_joint', 'right_finger_joint']

    def move_gripper(self, target_m: float, duration_sec: float = 1.0) -> bool:
        """Move both fingers to target_m (0.0=closed, 0.035=open)."""
        target_m = max(0.0, min(0.035, float(target_m)))

        self.get_logger().info('Waiting for /gripper_controller/follow_joint_trajectory...')
        if not self._client.wait_for_server(timeout_sec=4.0):
            self.get_logger().error('Gripper controller action server not available.')
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [target_m, target_m]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(
            seconds=int(duration_sec),
            nanoseconds=int((duration_sec % 1.0) * 1e9)
        ).to_msg()
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'Sending gripper command: {target_m:.4f} m...')
        send_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal was rejected.')
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Gripper position reached: {target_m:.4f} m.')
            return True
        else:
            self.get_logger().warn(f'Gripper motion ended with status: {result.status}')
            return False


def main(args=None):
    parser = argparse.ArgumentParser(description='Synora Gripper Command Utility')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--open', action='store_true', help='Fully open the gripper')
    group.add_argument('--close', action='store_true', help='Fully close the gripper')
    group.add_argument('--pos', type=float, help='Target finger position in metres (0.0 to 0.035)')
    parsed_args, ros_args = parser.parse_known_args(args=args if args is not None else sys.argv[1:])

    rclpy.init(args=ros_args)
    node = SynoraGripperController()

    try:
        if parsed_args.open:
            target = 0.035
        elif parsed_args.close:
            target = 0.0
        else:
            target = parsed_args.pos

        node.move_gripper(target)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
