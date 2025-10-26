#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_to_twist')
        # Publisher für cmd_vel
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publisher für Servos
        self.pub_servo1 = self.create_publisher(Int32, '/servo1/angle_cmd', 10)
        self.pub_servo2 = self.create_publisher(Int32, '/servo2/angle_cmd', 10)
        # self.pub_servo3 = self.create_publisher(Int32, '/servo3/angle_cmd', 10)
        # Initiale Servo-Winkel
        self.servo1_angle = 0
        self.servo2_angle = 0
        # self.servo2_angle = 0

        # Subscription auf /joy
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info('JoyToTwist Node gestartet.')

    def joy_callback(self, msg: Joy):
        # --- Twist mapping wie gehabt ---
        twist = Twist()
        twist.linear.x = -msg.axes[0] * 0.5
        twist.angular.z = -msg.axes[1] * 2.0
        self.pub_cmd.publish(twist)

        # --- D-Pad für Servos ---
        # Annahme: axes[7] = hoch/runter, axes[6] = links/rechts
        hat_vert = msg.axes[7]
        hat_horiz = msg.axes[6]

        # Servo1 steuern (hoch/runter)
        if hat_vert > 0.5:
            self.servo1_angle = min(self.servo1_angle + 25, 150)
            self.pub_servo1.publish(Int32(data=self.servo1_angle))
            self.get_logger().info(f'Servo1 ↑: {self.servo1_angle}°')
        elif hat_vert < -0.5:
            self.servo1_angle = max(self.servo1_angle - 25, -150)
            self.pub_servo1.publish(Int32(data=self.servo1_angle))
            self.get_logger().info(f'Servo1 ↓: {self.servo1_angle}°')

        # Servo2 steuern (links/rechts)
        if hat_horiz > 0.5:
            self.servo2_angle = min(self.servo2_angle + 25, 150)
            self.pub_servo2.publish(Int32(data=self.servo2_angle))
            self.get_logger().info(f'Servo2 →: {self.servo2_angle}°')
        elif hat_horiz < -0.5:
            self.servo2_angle = max(self.servo2_angle - 25, -150)
            self.pub_servo2.publish(Int32(data=self.servo2_angle))
            self.get_logger().info(f'Servo2 ←: {self.servo2_angle}°')
        
        # Servo3 steuern (auf/zu)
        #if hat_horiz > 0.5:
        #    self.servo3_angle = min(self.servo3_angle + 25, 150)
        #    self.pub_servo3.publish(Int32(data=self.servo3_angle))
        #    self.get_logger().info(f'Servo3 →: {self.servo3_angle}°')
        #elif hat_horiz < -0.5:
        #    self.servo3_angle = max(self.servo3_angle - 25, -150)
        #    self.pub_servo3.publish(Int32(data=self.servo3_angle))
        #    self.get_logger().info(f'Servo3 ←: {self.servo3_angle}°')

    def destroy_node(self):
        self.get_logger().info('Shutting down JoyToTwist node...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwist()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
