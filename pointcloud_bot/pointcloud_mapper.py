#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

# Fields we actually read/write, kept explicit so create_cloud never
# receives a field list that doesn't match the data.
CLOUD_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]

VOXEL_SIZE = 0.02  # meters; tune this — smaller = more detail, more points


class PointCloudMapper(Node):

    def __init__(self):
        super().__init__('pointcloud_mapper')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.pointcloud_callback,
            10
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            '/map/points',
            1
        )

        # voxel_key -> (x, y, z, rgb). A dict keyed by voxel index means
        # revisiting the same surface updates/keeps one point per voxel
        # instead of piling up duplicates — and nothing ever gets evicted
        # just because it's "old", so furniture stays once it's been seen.
        self.voxel_map = {}

        self.get_logger().info('Point cloud mapper started.')

    def pointcloud_callback(self, msg):
        try:
            if not self.tf_buffer.can_transform(
                'odom',
                msg.header.frame_id,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.5)
            ):
                return

            transform = self.tf_buffer.lookup_transform(
                'odom',
                msg.header.frame_id,
                msg.header.stamp
            )

            transformed_cloud = do_transform_cloud(msg, transform)

            points = point_cloud2.read_points(
                transformed_cloud,
                field_names=('x', 'y', 'z', 'rgb'),
                skip_nans=True
            )

            # Insert into the persistent voxel map. Points in the same
            # voxel overwrite each other (keeps memory/point-count bounded)
            # but the voxel itself is never removed just because time has
            # passed, so a couch scanned once and then left stays in the
            # map indefinitely.
            for x, y, z, rgb in points:
                key = (
                    int(np.floor(x / VOXEL_SIZE)),
                    int(np.floor(y / VOXEL_SIZE)),
                    int(np.floor(z / VOXEL_SIZE)),
                )
                self.voxel_map[key] = (x, y, z, rgb)

            header = transformed_cloud.header
            header.frame_id = 'odom'

            map_cloud = point_cloud2.create_cloud(
                header,
                CLOUD_FIELDS,
                list(self.voxel_map.values())
            )

            self.publisher.publish(map_cloud)

        except Exception as e:
            self.get_logger().warn(f'Could not process point cloud: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()