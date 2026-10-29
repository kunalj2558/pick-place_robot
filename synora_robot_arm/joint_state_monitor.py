#!/usr/bin/env python3
"""Live Joint State Monitor node for Synora 6-DOF Robot Arm."""

import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


ARM_LIMITS = {
    'shoulder_pan_joint': (-3.14159, 3.14159),
    'shoulder_lift_joint': (-2.0944, 2.0944),
    'elbow_joint': (-2.6179, 2.6179),
    'wrist_1_joint': (-3.14159, 3.14159),
    'wrist_2_joint': (-3.14159, 3.14159),
    'wrist_3_joint': (-3.14159, 3.14159),
}

GRIPPER_LIMITS = (0.0, 0.035)


class SynoraJointStateMonitor(Node):
    """Monitors and prints the joint states of the 6-DOF robot arm and gripper."""

    def __init__(self):
        super().__init__('synora_joint_state_monitor')
        self.declare_parameter('print_frequency_hz', 1.0)
        freq = self.get_parameter('print_frequency_hz').get_parameter_value().double_value

        self._latest_state = None
        self._sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._callback,
            10
        )
        self._timer = self.create_timer(1.0 / freq, self._print_table)
        self.get_logger().info('6-DOF Joint state monitor active. Listening to /joint_states...')

    def _callback(self, msg: JointState):
        self._latest_state = msg

    def _print_table(self):
        if self._latest_state is None:
            self.get_logger().info('Waiting for /joint_states topic...')
            return

        pos_dict = dict(zip(self._latest_state.name, self._latest_state.position))

        print('\033[H\033[J', end='')  # Clear terminal screen
        print('='*72)
        print('          SYNORA 6-DOF ROBOT ARM - REALTIME MONITOR')
        print('='*72)
        print(f' {"Joint Name":<22} | {"Pos (rad)":<10} | {"Pos (deg)":<10} | {"Status":<12}')
        print('-'*72)

        for j, (low, high) in ARM_LIMITS.items():
            if j in pos_dict:
                rad = pos_dict[j]
                deg = math.degrees(rad)
                status = 'OK' if (low <= rad <= high) else 'LIMIT EXCEEDED'
                print(f' {j:<22} | {rad:>9.3f}  | {deg:>9.1f}° | {status:<12}')

        print('-'*72)
        print(f' {"Gripper Joint":<22} | {"Pos (m)":<10} | {"Pos (mm)":<10} | {"Status":<12}')
        print('-'*72)
        for j in ['left_finger_joint', 'right_finger_joint']:
            if j in pos_dict:
                m = pos_dict[j]
                mm = m * 1000.0
                status = 'OK' if (GRIPPER_LIMITS[0] <= m <= GRIPPER_LIMITS[1] + 1e-3) else 'LIMIT EXCEEDED'
                print(f' {j:<22} | {m:>9.4f}  | {mm:>9.1f}mm | {status:<12}')

        if 'left_finger_joint' in pos_dict and 'right_finger_joint' in pos_dict:
            gap = (pos_dict['left_finger_joint'] + pos_dict['right_finger_joint']) * 1000.0 + 12.0
            print(f'\n Total Gripper Opening Gap: {gap:.1f} mm')
        print('='*72)


def main(args=None):
    rclpy.init(args=args)
    node = SynoraJointStateMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
