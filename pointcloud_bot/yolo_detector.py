import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

import cv2
import numpy as np


class YOLODetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        self.bridge = CvBridge()

        # Load pretrained YOLO model
        self.model = YOLO('yolo26n.pt')

        # Camera image subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        # Annotated image publisher
        self.image_publisher = self.create_publisher(
            Image,
            '/camera/yolo_image',
            10
        )

        self.get_logger().info(
            'YOLO detector started.'
        )

        self.get_logger().info(
            'Input: /camera/image'
        )

        self.get_logger().info(
            'Output: /camera/yolo_image'
        )

    def image_callback(self, msg):

        try:

            # ROS Image -> OpenCV image
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:

            self.get_logger().error(
                f'Image conversion failed: {e}'
            )

            return

        # Make sure NumPy array is contiguous
        frame = np.ascontiguousarray(frame)

        # Run YOLO
        results = self.model(
            frame,
            verbose=False
        )

        # Process detections
        for result in results:

            for box in result.boxes:

                x1, y1, x2, y2 = map(
                    int,
                    box.xyxy[0].tolist()
                )

                confidence = float(
                    box.conf[0]
                )

                class_id = int(
                    box.cls[0]
                )

                class_name = result.names[class_id]

                # Print detection
                self.get_logger().info(
                    f'Detected: {class_name} '
                    f'confidence={confidence:.2f} '
                    f'bbox=['
                    f'{x1}, '
                    f'{y1}, '
                    f'{x2}, '
                    f'{y2}]'
                )

                # Draw bounding box
                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    (255, 0, 0),
                    2
                )

                # Label
                label = (
                    f'{class_name} '
                    f'{confidence:.2f}'
                )

                cv2.putText(
                    frame,
                    label,
                    (x1, max(y1 - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

        # OpenCV -> ROS Image
        try:

            output_msg = Image()

            output_msg.header = msg.header
            output_msg.height = frame.shape[0]
            output_msg.width = frame.shape[1]
            output_msg.encoding = 'bgr8'
            output_msg.is_bigendian = 0
            output_msg.step = frame.shape[1] * 3
            output_msg.data = frame.tobytes()

            self.image_publisher.publish(output_msg)

        except Exception as e:

            self.get_logger().error(
                f'Output image conversion failed: {repr(e)}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = YOLODetector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()