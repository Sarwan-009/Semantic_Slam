#!/usr/bin/env python3

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Speeds
        self.linear_speed = 0.5
        self.angular_speed = 1.0

        self.get_logger().info('Teleop node started')

        self.print_instructions()

    def print_instructions(self):

        print()
        print('====================================')
        print('        ROS 2 Robot Teleop')
        print('====================================')
        print()
        print('   W : Forward')
        print('   S : Backward')
        print('   A : Turn Left')
        print('   D : Turn Right')
        print('   X : Stop')
        print()
        print('   Q : Increase linear speed')
        print('   Z : Decrease linear speed')
        print()
        print('   CTRL+C : Exit')
        print()
        print('====================================')
        print()

    def get_key(self):

        settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                settings
            )

        return key

    def run(self):

        twist = Twist()

        try:

            while rclpy.ok():

                key = self.get_key()

                # --------------------------------
                # Forward
                # --------------------------------
                if key.lower() == 'w':

                    twist.linear.x = self.linear_speed
                    twist.angular.z = 0.0

                # --------------------------------
                # Backward
                # --------------------------------
                elif key.lower() == 's':

                    twist.linear.x = -self.linear_speed
                    twist.angular.z = 0.0

                # --------------------------------
                # Left
                # --------------------------------
                elif key.lower() == 'a':

                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed

                # --------------------------------
                # Right
                # --------------------------------
                elif key.lower() == 'd':

                    twist.linear.x = 0.0
                    twist.angular.z = -self.angular_speed

                # --------------------------------
                # Stop
                # --------------------------------
                elif key.lower() == 'x':

                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                # --------------------------------
                # Increase speed
                # --------------------------------
                elif key.lower() == 'q':

                    self.linear_speed += 0.1

                    print(
                        f'\nLinear speed: '
                        f'{self.linear_speed:.2f}'
                    )

                    continue

                # --------------------------------
                # Decrease speed
                # --------------------------------
                elif key.lower() == 'z':

                    self.linear_speed = max(
                        0.1,
                        self.linear_speed - 0.1
                    )

                    print(
                        f'\nLinear speed: '
                        f'{self.linear_speed:.2f}'
                    )

                    continue

                else:
                    continue

                self.publisher.publish(twist)

                print(
                    f'\rLinear: {twist.linear.x:.2f} '
                    f'Angular: {twist.angular.z:.2f}',
                    end='',
                    flush=True
                )

        except KeyboardInterrupt:
            pass

        finally:

            # Stop robot before exiting
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            self.publisher.publish(twist)


def main(args=None):

    rclpy.init(args=args)

    node = TeleopNode()

    try:
        node.run()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()