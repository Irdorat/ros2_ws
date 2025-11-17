#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import os
import yaml

class EpisodeRecorder(Node):
    def __init__(self, episode_name='episode_001'):
        super().__init__('episode_recorder')

        # Создаём папки для эпизода
        self.base_path = os.path.join(os.getcwd(), episode_name)
        os.makedirs(os.path.join(self.base_path, 'images'), exist_ok=True)
        os.makedirs(os.path.join(self.base_path, 'odom'), exist_ok=True)

        self.bridge = CvBridge()
        self.counter = 0
        self.episode_data = []

        # Подписка на камеру
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Подписка на одометрию
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Таймер для нумерации кадров
        self.timer = self.create_timer(0.1, self.timer_cb)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_path = os.path.join(self.base_path, 'images', f'{self.counter:05d}.png')
        cv2.imwrite(img_path, cv_image)
        self.get_logger().info(f"Saved image {img_path}")

    def odom_callback(self, msg):
        odom_info = {
            'counter': self.counter,
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z,
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w,
            }
        }
        self.episode_data.append(odom_info)

    def timer_cb(self):
        self.counter += 1

    def save_episode(self):
        yaml_path = os.path.join(self.base_path, 'odom.yaml')
        with open(yaml_path, 'w') as f:
            yaml.dump(self.episode_data, f)
        self.get_logger().info(f"Episode data saved to {yaml_path}")

def main(args=None):
    rclpy.init(args=args)
    recorder = EpisodeRecorder('episode_001')

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info("Shutting down...")
    finally:
        recorder.save_episode()
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
