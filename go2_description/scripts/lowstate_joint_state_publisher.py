#!/usr/bin/env python3

import math

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState


DEFAULT_JOINT_NAMES = [
    'FR_hip_joint',
    'FR_thigh_joint',
    'FR_calf_joint',
    'FL_hip_joint',
    'FL_thigh_joint',
    'FL_calf_joint',
    'RR_hip_joint',
    'RR_thigh_joint',
    'RR_calf_joint',
    'RL_hip_joint',
    'RL_thigh_joint',
    'RL_calf_joint',
]

DEFAULT_MOTOR_INDICES = list(range(12))


class LowStateJointStatePublisher(Node):
    def __init__(self):
        super().__init__('lowstate_joint_state_publisher')

        self.declare_parameter('lowstate_topic', '/lowstate')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('motor_indices', DEFAULT_MOTOR_INDICES)
        self.declare_parameter('publish_initial_state', True)
        self.declare_parameter('initial_state_publish_rate', 10.0)

        lowstate_topic = self.get_parameter('lowstate_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.motor_indices = [int(index) for index in self.get_parameter('motor_indices').value]
        self.received_lowstate = False
        self.shutting_down = False
        self.initial_positions = [0.0] * len(self.joint_names)
        self.initial_velocities = [0.0] * len(self.joint_names)
        self.publisher = None
        self.subscription = None
        self.initial_state_timer = None

        if len(self.joint_names) != len(self.motor_indices):
            raise ValueError(
                'joint_names and motor_indices must have the same length '
                f'({len(self.joint_names)} != {len(self.motor_indices)})'
            )

        self.publisher = self.create_publisher(JointState, joint_states_topic, 10)
        self.subscription = self.create_subscription(
            LowState,
            lowstate_topic,
            self.lowstate_callback,
            qos_profile_sensor_data,
        )

        if self.get_parameter('publish_initial_state').value:
            initial_state_publish_rate = float(self.get_parameter('initial_state_publish_rate').value)
            if initial_state_publish_rate > 0.0:
                self.initial_state_timer = self.create_timer(
                    1.0 / initial_state_publish_rate,
                    self.publish_initial_state,
                    clock=Clock(clock_type=ClockType.STEADY_TIME),
                )
            self.publish_initial_state()

        self.get_logger().info(
            f'Publishing {joint_states_topic} from {lowstate_topic} '
            f'for {len(self.joint_names)} GO2 joints'
        )

    def lowstate_callback(self, msg):
        if self.shutting_down:
            return

        positions = []
        velocities = []

        for index in self.motor_indices:
            if index >= len(msg.motor_state):
                self.get_logger().warn(
                    f'LowState motor_state has only {len(msg.motor_state)} entries; '
                    f'cannot read index {index}',
                    throttle_duration_sec=2.0,
                )
                return

            motor = msg.motor_state[index]
            positions.append(self.finite_or_zero(motor.q))
            velocities.append(self.finite_or_zero(motor.dq))

        self.received_lowstate = True
        self.publish_joint_state(positions, velocities)

    def publish_initial_state(self):
        if self.shutting_down:
            return

        if self.received_lowstate:
            if self.initial_state_timer is not None:
                self.initial_state_timer.cancel()
                self.destroy_timer(self.initial_state_timer)
                self.initial_state_timer = None
            return

        self.publish_joint_state(self.initial_positions, self.initial_velocities)

    def publish_joint_state(self, positions, velocities):
        if self.shutting_down or self.publisher is None or not self.context.ok():
            return

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = positions
        joint_state.velocity = velocities
        self.publisher.publish(joint_state)

    @staticmethod
    def finite_or_zero(value):
        value = float(value)
        return value if math.isfinite(value) else 0.0

    def cleanup(self):
        self.shutting_down = True

        if self.initial_state_timer is not None:
            self.initial_state_timer.cancel()
            self.destroy_timer(self.initial_state_timer)
            self.initial_state_timer = None

        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None

        if self.publisher is not None:
            self.destroy_publisher(self.publisher)
            self.publisher = None


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.ALL)
    node = None
    try:
        node = LowStateJointStatePublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if node.context.ok():
                node.get_logger().info('Shutting down lowstate joint state publisher')
            node.cleanup()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
