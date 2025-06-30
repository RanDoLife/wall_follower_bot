#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

class WallFollowerBot(Node):
    def __init__(self):
        super().__init__('wall_follower_bot')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/robot_state', 10)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.control_loop)

        self.state = 'search_wall'
        self.front_dist = float('inf')
        self.front_right_dist = float('inf')
        self.right_dist = float('inf')
        self.back_right_dist = float('inf')

        self.see_bin = False
        self.bin_distance = None

        # PID параметры
        self.target_dist = 0.4  # Желаемое расстояние до стены
        self.kp = 1.2
        self.ki = 0.0
        self.kd = 0.5

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges = np.clip(ranges, msg.range_min, msg.range_max)

        # Углы лидара
        self.front_dist = np.mean(ranges[0:10].tolist() + ranges[-10:].tolist())
        self.front_right_dist = np.mean(ranges[330:340])
        self.right_dist = np.mean(ranges[260:280])
        self.back_right_dist = np.mean(ranges[200:220])

        if self.see_bin:
            self.bin_distance = self.front_dist

    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # === ОБРЕЗКА ЦЕНТРАЛЬНОЙ ОБЛАСТИ ===
        h, w, _ = img.shape
        center_fraction = 0.3  # используем 30% по ширине (можно 0.2, 0.4 — настраивается)

        x_start = int(w * (0.5 - center_fraction / 2))
        x_end = int(w * (0.5 + center_fraction / 2))
        img_cropped = img[:, x_start:x_end]

        hsv = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2HSV)

        # Маска под тёмно-зелёный бак (суженный диапазон)
        lower = np.array([55, 170, 0])
        upper = np.array([65, 255, 30])

        mask = cv2.inRange(hsv, lower, upper)
        bin_area = cv2.countNonZero(mask)

        self.see_bin = bin_area > 1000

    def publish_state(self, message: str):
        msg = String()
        msg.data = message
        self.state_pub.publish(msg)
        self.get_logger().info(message)

    def pid_control(self, error):
        current_time = time.time()
        dt = current_time - self.prev_time if self.prev_time else 0.1

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error
        self.prev_time = current_time

        return output

    def control_loop(self):
        twist = Twist()

        if self.state == 'search_wall':
            self.publish_state('🔍 Поиск стены...')
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            if self.front_dist < 1.5:
                self.state = 'approach_wall'

        elif self.state == 'approach_wall':
            self.publish_state(f'➡️ Подъезд к стене... (дистанция {self.front_dist:.2f} м)')
            if self.front_dist > self.target_dist:
                twist.linear.x = 0.1
            else:
                self.state = 'follow_wall'

        elif self.state == 'follow_wall':
            if self.see_bin and self.front_dist < 0.6:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publish_state(f'🛑 Бак найден. Остановка на расстоянии {self.front_dist:.2f} м')
                self.state = 'stop'
            else:
                front = self.front_dist < 0.5
                front_right = self.front_right_dist < 0.5* 1.4
                right = self.right_dist < 0.45
                back_right = self.back_right_dist < 0.45* 1.5

                if front:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                    self.publish_state('🔄 Стена спереди — поворачиваем влево')

                elif not right and not front_right:
                    twist.linear.x = 0.0
                    twist.angular.z = -0.4
                    self.publish_state('↪️ Стена справа пропала — поворот направо')

                elif right and not back_right:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.3
                    self.publish_state('↩️ Поворот стены влево — поворачиваем')

                else:
                    error = self.right_dist - self.target_dist
                    correction = -self.pid_control(error)
                    twist.linear.x = 0.15
                    twist.angular.z = correction
                    self.publish_state(f'➡️ Следуем вдоль стены. Ошибка: {error:.2f}, коррекция: {correction:.2f}')

        elif self.state == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerBot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
