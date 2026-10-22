import math

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs

from ultralytics import YOLO


class YOLO3DMapper(Node):

    def __init__(self):
        super().__init__('yolo_3d_mapper')

        # ==================================================
        # YOLO
        # ==================================================

        self.model = YOLO('yolov8n.pt')

        self.bridge = CvBridge()

        # ==================================================
        # TF2
        # ==================================================

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ==================================================
        # Point Cloud
        # ==================================================

        self.latest_pointcloud = None

        # ==================================================
        # Persistent Semantic Landmarks
        # ==================================================

        self.landmarks = {}

        # Maximum distance for associating a new detection
        # with an existing landmark
        self.association_distance = 0.75

        # Observation count for every landmark
        self.observation_count = {}

        # Marker IDs
        self.marker_ids = {}

        self.next_marker_id = 0

        # ==================================================
        # Subscribers
        # ==================================================

        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.pointcloud_callback,
            10
        )

        # ==================================================
        # Publishers
        # ==================================================

        self.image_publisher = self.create_publisher(
            Image,
            '/camera/yolo_3d_image',
            10
        )

        self.marker_publisher = self.create_publisher(
            Marker,
            '/semantic_objects',
            10
        )

        self.get_logger().info(
            'YOLO 3D Semantic Mapper started.'
        )

    # ======================================================
    # POINT CLOUD CALLBACK
    # ======================================================

    def pointcloud_callback(self, msg):

        self.latest_pointcloud = msg

    # ======================================================
    # GET ROBUST 3D POSITION
    # ======================================================

    def get_robust_3d_position(self, cx, cy):

        if self.latest_pointcloud is None:
            return None

        cloud = self.latest_pointcloud

        # Convert organized PointCloud2 into a list
        points = list(
            point_cloud2.read_points(
                cloud,
                field_names=('x', 'y', 'z'),
                skip_nans=False
            )
        )

        radius = 5

        x_values = []
        y_values = []
        z_values = []

        # Search an 11 x 11 region around
        # the center of the YOLO bounding box
        for v in range(
            max(0, cy - radius),
            min(cloud.height, cy + radius + 1)
        ):

            for u in range(
                max(0, cx - radius),
                min(cloud.width, cx + radius + 1)
            ):

                index = v * cloud.width + u

                if index >= len(points):
                    continue

                x, y, z = points[index]

                # Reject invalid depth values
                if (
                    math.isfinite(x)
                    and math.isfinite(y)
                    and math.isfinite(z)
                ):

                    x_values.append(float(x))
                    y_values.append(float(y))
                    z_values.append(float(z))

        # No valid depth points
        if len(x_values) == 0:
            return None

        # Median gives a more robust estimate
        # than using a single pixel
        X = float(np.median(x_values))
        Y = float(np.median(y_values))
        Z = float(np.median(z_values))

        return X, Y, Z

    # ======================================================
    # TRANSFORM POINT USING TF2
    # ======================================================

    def transform_point(
        self,
        X,
        Y,
        Z,
        source_frame,
        target_frame
    ):

        point = PointStamped()

        point.header.frame_id = source_frame

        point.header.stamp = self.get_clock().now().to_msg()

        point.point.x = X
        point.point.y = Y
        point.point.z = Z

        try:

            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )

            transformed_point = (
                tf2_geometry_msgs.do_transform_point(
                    point,
                    transform
                )
            )

            return (
                transformed_point.point.x,
                transformed_point.point.y,
                transformed_point.point.z
            )

        except Exception as e:

            self.get_logger().warn(
                f'TF transform failed: '
                f'{source_frame} -> {target_frame}: {e}'
            )

            return None

    # ======================================================
    # DATA ASSOCIATION
    # ======================================================

    def associate_landmark(
        self,
        class_name,
        position
    ):

        X, Y, Z = position

        best_id = None

        best_distance = float('inf')

        # --------------------------------------------------
        # Search existing landmarks
        # --------------------------------------------------

        for landmark_id, landmark in self.landmarks.items():

            # Only compare the same object class
            if landmark['class'] != class_name:
                continue

            lx = landmark['position'][0]
            ly = landmark['position'][1]
            lz = landmark['position'][2]

            distance = math.sqrt(
                (X - lx) ** 2 +
                (Y - ly) ** 2 +
                (Z - lz) ** 2
            )

            if distance < best_distance:

                best_distance = distance
                best_id = landmark_id

        # --------------------------------------------------
        # Existing landmark
        # --------------------------------------------------

        if (
            best_id is not None
            and best_distance < self.association_distance
        ):

            old_position = (
                self.landmarks[best_id]['position']
            )

            # Exponential smoothing
            alpha = 0.2

            new_position = (

                old_position[0] * (1 - alpha)
                + X * alpha,

                old_position[1] * (1 - alpha)
                + Y * alpha,

                old_position[2] * (1 - alpha)
                + Z * alpha
            )

            self.landmarks[best_id]['position'] = (
                new_position
            )

            self.observation_count[best_id] += 1

            return best_id, False

        # --------------------------------------------------
        # New landmark
        # --------------------------------------------------

        landmark_number = len(self.landmarks)

        landmark_id = (
            f'{class_name}_{landmark_number}'
        )

        self.landmarks[landmark_id] = {

            'class': class_name,

            'position': (
                X,
                Y,
                Z
            )
        }

        self.observation_count[landmark_id] = 1

        return landmark_id, True

    # ======================================================
    # GET RVIZ MARKER ID
    # ======================================================

    def get_marker_id(self, landmark_id):

        if landmark_id not in self.marker_ids:

            self.marker_ids[landmark_id] = (
                self.next_marker_id
            )

            self.next_marker_id += 1

        return self.marker_ids[landmark_id]

    # ======================================================
    # PUBLISH RVIZ MARKER
    # ======================================================

    def publish_marker(
        self,
        landmark_id,
        class_name,
        position
    ):

        X, Y, Z = position

        marker = Marker()

        # Marker exists in odom frame
        marker.header.frame_id = 'odom'

        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = 'semantic_objects'

        marker.id = self.get_marker_id(
            landmark_id
        )

        marker.type = Marker.SPHERE

        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = X
        marker.pose.position.y = Y
        marker.pose.position.z = Z

        marker.pose.orientation.w = 1.0

        # Sphere size
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        # Marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2

        # Keep marker permanently visible
        marker.lifetime.sec = 0

        self.marker_publisher.publish(marker)

    # ======================================================
    # IMAGE CALLBACK
    # ======================================================

    def image_callback(self, msg):

        # Need point cloud before doing 3D localization
        if self.latest_pointcloud is None:
            return

        # --------------------------------------------------
        # Convert ROS Image → OpenCV
        # --------------------------------------------------

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:

            self.get_logger().error(
                f'Image conversion failed: {e}'
            )

            return

        # --------------------------------------------------
        # Run YOLO
        # --------------------------------------------------

        results = self.model(
            frame,
            verbose=False
        )

        # --------------------------------------------------
        # Process detections
        # --------------------------------------------------

        for result in results:

            if result.boxes is None:
                continue

            for box in result.boxes:

             
                # Bounding Box
          

                x1, y1, x2, y2 = (
                    box.xyxy[0]
                    .cpu()
                    .numpy()
                )

                x1 = int(x1)
                y1 = int(y1)
                x2 = int(x2)
                y2 = int(y2)

      
                # Confidence
       

                confidence = float(
                    box.conf[0]
                    .cpu()
                    .numpy()
                )

          
                # Class
     

                class_id = int(
                    box.cls[0]
                    .cpu()
                    .numpy()
                )

                class_name = self.model.names[
                    class_id
                ]


                # Bounding Box Center


                cx = int(
                    (x1 + x2) / 2
                )

                cy = int(
                    (y1 + y2) / 2
                )


                # Get 3D Camera Position


                camera_position = (
                    self.get_robust_3d_position(
                        cx,
                        cy
                    )
                )

                # Draw Bounding Box
     

                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )


                # Draw Center


                cv2.circle(
                    frame,
                    (cx, cy),
                    5,
                    (0, 0, 255),
                    -1
                )

                # No Depth


                if camera_position is None:

                    cv2.putText(
                        frame,
                        f'{class_name} - no depth',
                        (
                            x1,
                            max(20, y1 - 10)
                        ),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        2
                    )

                    continue


                # Camera Coordinates
 

                cam_X, cam_Y, cam_Z = (
                    camera_position
                )


                # camera_link → base_link


                base_position = (
                    self.transform_point(
                        cam_X,
                        cam_Y,
                        cam_Z,
                        'camera_link',
                        'base_link'
                    )
                )

                if base_position is None:
                    continue

                base_X, base_Y, base_Z = (
                    base_position
                )


                # base_link → odom

                odom_position = (
                    self.transform_point(
                        base_X,
                        base_Y,
                        base_Z,
                        'base_link',
                        'odom'
                    )
                )

                if odom_position is None:
                    continue

                odom_X, odom_Y, odom_Z = (
                    odom_position
                )

                # Associate With Semantic Landmark

                landmark_id, is_new = (
                    self.associate_landmark(
                        class_name,
                        odom_position
                    )
                )

                # Get smoothed landmark position
                landmark_position = (
                    self.landmarks[
                        landmark_id
                    ]['position']
                )

                # Publish RViz Marker


                self.publish_marker(
                    landmark_id,
                    class_name,
                    landmark_position
                )
               
                # Log New Landmark
                
                if is_new:

                    self.get_logger().info(
                        f'NEW LANDMARK: '
                        f'{landmark_id} | '
                        f'position='
                        f'{landmark_position}'
                    )

                else:

                    self.get_logger().info(
                        f'UPDATED LANDMARK: '
                        f'{landmark_id} | '
                        f'position='
                        f'{landmark_position} | '
                        f'observations='
                        f'{self.observation_count[landmark_id]}'
                    )
                
                # Draw Information On Image

                text1 = (
                    f'{landmark_id} '
                    f'{confidence:.2f}'
                )

                text2 = (
                    f'X:{landmark_position[0]:.2f} '
                    f'Y:{landmark_position[1]:.2f} '
                    f'Z:{landmark_position[2]:.2f}'
                )

                cv2.putText(
                    frame,
                    text1,
                    (
                        x1,
                        max(20, y1 - 30)
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    text2,
                    (
                        x1,
                        max(40, y1 - 10)
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 255, 0),
                    2
                )

        # Publish Annotated Image
        
        output_msg = Image()

        output_msg.header = msg.header

        output_msg.height = frame.shape[0]

        output_msg.width = frame.shape[1]

        output_msg.encoding = 'bgr8'

        output_msg.is_bigendian = 0

        output_msg.step = (
            frame.shape[1] * 3
        )

        output_msg.data = frame.tobytes()

        self.image_publisher.publish(
            output_msg
        )

# MAIN


def main(args=None):

    rclpy.init(args=args)

    node = YOLO3DMapper()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()