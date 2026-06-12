#!/usr/bin/env python3

import struct

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


POINT_STEP = 26


class LivoxCustomToPointCloud2(Node):
    def __init__(self):
        super().__init__('livox_custom_to_pointcloud2')

        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/points')
        self.declare_parameter('frame_id', 'mid360_lidar')
        self.declare_parameter('use_message_stamp', False)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.use_message_stamp = self.as_bool(self.get_parameter('use_message_stamp').value)

        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self.subscription = self.create_subscription(
            CustomMsg,
            input_topic,
            self.custom_callback,
            qos_profile_sensor_data,
        )

        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='tag', offset=16, datatype=PointField.UINT8, count=1),
            PointField(name='line', offset=17, datatype=PointField.UINT8, count=1),
            PointField(name='timestamp', offset=18, datatype=PointField.FLOAT64, count=1),
        ]

        self.get_logger().info(
            f'Converting {input_topic} CustomMsg to {output_topic} PointCloud2 in frame {self.frame_id}'
        )

    def custom_callback(self, msg):
        point_count = int(msg.point_num) if msg.point_num else len(msg.points)
        point_count = min(point_count, len(msg.points))

        cloud = PointCloud2()
        cloud.header.stamp = msg.header.stamp if self.use_message_stamp else self.get_clock().now().to_msg()
        cloud.header.frame_id = self.frame_id
        cloud.height = 1
        cloud.width = point_count
        cloud.fields = self.fields
        cloud.is_bigendian = False
        cloud.point_step = POINT_STEP
        cloud.row_step = POINT_STEP * point_count
        cloud.is_dense = True

        data = bytearray(cloud.row_step)
        for i, point in enumerate(msg.points[:point_count]):
            offset = i * POINT_STEP
            struct.pack_into(
                '<ffffBBd',
                data,
                offset,
                float(point.x),
                float(point.y),
                float(point.z),
                float(point.reflectivity),
                int(point.tag),
                int(point.line),
                float(point.offset_time),
            )

        cloud.data = bytes(data)
        self.publisher.publish(cloud)

    @staticmethod
    def as_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ('1', 'true', 'yes', 'on')
        return bool(value)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxCustomToPointCloud2()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
