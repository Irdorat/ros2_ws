import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import numpy as np, cv2, math

class TestSensors(Node):
    def __init__(self):
        super().__init__('test_sensors')
        self.pub_img = self.create_publisher(Image, '/camera/image_raw', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.count = 0
    def timer_cb(self):
        t = self.get_clock().now().nanoseconds / 1e9 - self.t0
        h,w = 240,320
        img = np.zeros((h,w,3), dtype=np.uint8)
        cx = int((w/2) + math.sin(t*0.8)*80)
        cy = int((h/2) + math.cos(t*0.6)*40)
        cv2.circle(img, (cx,cy), 30, (0,255,255), -1)
        cv2.putText(img, f'frame {self.count}', (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        self.pub_img.publish(msg)
        # odom
        from nav_msgs.msg import Odometry
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = 1.0 * math.cos(0.4*t)
        odom.pose.pose.position.y = 1.0 * math.sin(0.4*t)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(odom)
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()