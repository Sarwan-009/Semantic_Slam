import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

import tf2_ros


class PointCloudMapper(Node):

    def __init__(self):
        super().__init__('pointcloud_mapper')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar_3d/points',
            self.pointcloud_callback,
            10
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            '/global_pointcloud',
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.cloud_count = 0

        self.get_logger().info(
            '3D LiDAR point-cloud mapper started.'
        )

    def quaternion_to_rotation_matrix(self, q):

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        R = np.array([
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w)
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w)
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y)
            ]
        ], dtype=np.float32)

        return R

    def pointcloud_callback(self, msg):

        try:

            # --------------------------------------------------
            # 1. Get LiDAR -> odom transform
            # --------------------------------------------------

            if not self.tf_buffer.can_transform(
                'odom',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(
                    seconds=0.05
                )
            ):
                return

            transform = self.tf_buffer.lookup_transform(
                'odom',
                msg.header.frame_id,
                rclpy.time.Time()
            )

            # --------------------------------------------------
            # 2. Read XYZ points
            #
            # ROS 2 Jazzy returns an ordinary Nx3 float32 array
            # for read_points_numpy()
            # --------------------------------------------------

            points = point_cloud2.read_points_numpy(
                msg,
                field_names=['x', 'y', 'z'],
                skip_nans=True
            )

            if points is None or points.size == 0:
                return

            points = np.asarray(
                points,
                dtype=np.float32
            ).reshape(-1, 3)

            # --------------------------------------------------
            # 3. Rotation + translation
            # --------------------------------------------------

            R = self.quaternion_to_rotation_matrix(
                transform.transform.rotation
            )

            t = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ], dtype=np.float32)

            # --------------------------------------------------
            # 4. Transform points into odom frame
            # --------------------------------------------------

            transformed_points = (
                points @ R.T
            ) + t

            # --------------------------------------------------
            # 5. Create clean PointCloud2
            # --------------------------------------------------

            header = msg.header
            header.frame_id = 'odom'

            fields = [
                PointField(
                    name='x',
                    offset=0,
                    datatype=PointField.FLOAT32,
                    count=1
                ),
                PointField(
                    name='y',
                    offset=4,
                    datatype=PointField.FLOAT32,
                    count=1
                ),
                PointField(
                    name='z',
                    offset=8,
                    datatype=PointField.FLOAT32,
                    count=1
                )
            ]

            output_cloud = point_cloud2.create_cloud(
                header,
                fields,
                transformed_points.tolist()
            )

            self.publisher.publish(output_cloud)

            # --------------------------------------------------
            # 6. Logging
            # --------------------------------------------------

            self.cloud_count += 1

            if self.cloud_count % 10 == 0:

                self.get_logger().info(
                    f'Processed {self.cloud_count} scans | '
                    f'{len(transformed_points)} points/scan'
                )

        except Exception as e:

            self.get_logger().warn(
                f'Could not transform point cloud: {e}'
            )


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