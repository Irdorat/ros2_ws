import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import os
import json
import yaml
import time
from pathlib import Path
import cv2

class EpisodeRecorder(Node):
    def __init__(self, episode_name='episode_001'):
        super().__init__('episode_recorder')
        # fixed base path (explicit)
        self.base_path = Path(os.path.expanduser(f'~/ros2_ws/episodes/{episode_name}'))
        (self.base_path / 'rgb').mkdir(parents=True, exist_ok=True)
        (self.base_path / 'depth').mkdir(parents=True, exist_ok=True)
        (self.base_path / 'pcd').mkdir(parents=True, exist_ok=True)
        (self.base_path / 'imu').mkdir(parents=True, exist_ok=True)
        (self.base_path / 'dsg').mkdir(parents=True, exist_ok=True)
        (self.base_path / 'tf').mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.counter = 0

        # buffers for synchronization (simple latest-buffer approach)
        self.buff = {'rgb': None, 'depth': None, 'pcd': None, 'imu': None, 'dsg': None, 'odom': None}

        # subscriptions
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        # if you have depth topic, add it similarly:
        # self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(MarkerArray, '/dsg_markers', self.dsg_callback, 10)
        self.create_subscription(PointCloud2, '/camera/points', self.pc_cb, 1)

        # sync timer - every N seconds try to write a synchronized frame
        self.sync_timer = self.create_timer(0.1, self.sync_and_save)
        # meta index file
        self.meta_file = self.base_path / 'meta.csv'
        if not self.meta_file.exists():
            with self.meta_file.open('w') as f:
                f.write('frame_idx,ts_sec,ts_nsec,rgb,depth,pcd,imu,dsg,odom\n')

        self.get_logger().info(f"EpisodeRecorder ready, saving to {str(self.base_path)}")

    # ---------- helpers ----------
    def _stamp_to_str(self, stamp):
        return f"{stamp.sec}_{stamp.nanosec}"

    def save_markerarray(self, marker_array: MarkerArray, out_path: Path):
        data = []
        for m in marker_array.markers:
            entry = {
                'header': {
                    'stamp_sec': int(m.header.stamp.sec),
                    'stamp_nsec': int(m.header.stamp.nanosec),
                    'frame_id': m.header.frame_id
                },
                'ns': m.ns,
                'id': int(m.id),
                'type': int(m.type),
                'action': int(m.action),
                'pose': {
                    'position': {
                        'x': float(m.pose.position.x),
                        'y': float(m.pose.position.y),
                        'z': float(m.pose.position.z)
                    },
                    'orientation': {
                        'x': float(m.pose.orientation.x),
                        'y': float(m.pose.orientation.y),
                        'z': float(m.pose.orientation.z),
                        'w': float(m.pose.orientation.w)
                    }
                },
                'scale': {'x': float(m.scale.x), 'y': float(m.scale.y), 'z': float(m.scale.z)},
                'color': {'r': float(m.color.r), 'g': float(m.color.g), 'b': float(m.color.b), 'a': float(m.color.a)},
                'text': getattr(m, 'text', '')
            }
            data.append(entry)
        try:
            out_path.parent.mkdir(parents=True, exist_ok=True)
            with out_path.open('w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Failed save_markerarray {out_path}: {e}")

    def save_pointcloud2_to_pcd(self, pc2_msg: PointCloud2, filename: Path):
        """Stream points from message to ASCII PCD file (only x,y,z)"""
        try:
            it = pc2.read_points(pc2_msg, field_names=('x','y','z'), skip_nans=True)
            # write header
            with filename.open('w') as f:
                f.write('# .PCD v0.7 - Point Cloud Data file\n')
                f.write('VERSION 0.7\n')
                f.write('FIELDS x y z\n')
                f.write('SIZE 4 4 4\n')
                f.write('TYPE F F F\n')
                f.write('COUNT 1 1 1\n')
                # we don't know width until we iterate, so buffer lines to temp file or count first pass
                points = list(it)  # for prototyping — if large clouds, change to two-pass streaming
                f.write(f'WIDTH {len(points)}\n')
                f.write('HEIGHT 1\n')
                f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
                f.write(f'POINTS {len(points)}\n')
                f.write('DATA ascii\n')
                for p in points:
                    f.write(f'{p[0]} {p[1]} {p[2]}\n')
        except Exception as e:
            self.get_logger().error(f"Failed save_pointcloud2_to_pcd {filename}: {e}")

    # ---------- callbacks ----------
    def image_callback(self, msg: Image):
        self.buff['rgb'] = msg
        # keep also raw quick save if desired (but rely on sync to actually flush to disk)

    def odom_callback(self, msg: Odometry):
        self.buff['odom'] = msg

    def dsg_callback(self, msg: MarkerArray):
        self.buff['dsg'] = msg

    def pc_cb(self, msg: PointCloud2):
        self.buff['pcd'] = msg

    # ---------- synchronization and saving ----------
    def sync_and_save(self):
        # decide master timestamp — latest common stamp among non-None buffers
        # here simple logic: if we have rgb and odom, save a frame using rgb stamp
        rgb = self.buff.get('rgb')
        if rgb is None:
            return  # nothing to save yet
        stamp = rgb.header.stamp
        ts_sec, ts_nsec = int(stamp.sec), int(stamp.nanosec)
        idx = self.counter
        # filenames
        rgb_fname = self.base_path / 'rgb' / f'{idx:06d}_{ts_sec}_{ts_nsec}.png'
        pcd_fname = None
        dsg_fname = None
        odom_fname = self.base_path / 'odom' / f'{idx:06d}_{ts_sec}_{ts_nsec}.yaml'

        # save RGB
        try:
            cv_img = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
            cv2.imwrite(str(rgb_fname), cv_img)
        except Exception as e:
            self.get_logger().error(f"Failed to write RGB: {e}")
            return

        # save odom if present
        od = self.buff.get('odom')
        if od is not None:
            od_dict = {
                'position': {
                    'x': float(od.pose.pose.position.x),
                    'y': float(od.pose.pose.position.y),
                    'z': float(od.pose.pose.position.z),
                },
                'orientation': {
                    'x': float(od.pose.pose.orientation.x),
                    'y': float(od.pose.pose.orientation.y),
                    'z': float(od.pose.pose.orientation.z),
                    'w': float(od.pose.pose.orientation.w),
                }
            }
            with odom_fname.open('w') as f:
                yaml.dump(od_dict, f)

        # save pointcloud if present
        pc = self.buff.get('pcd')
        if pc is not None:
            pcd_fname = self.base_path / 'pcd' / f'{idx:06d}_{ts_sec}_{ts_nsec}.pcd'
            self.save_pointcloud2_to_pcd(pc, pcd_fname)

        # save dsg if present
        dsg = self.buff.get('dsg')
        if dsg is not None:
            dsg_fname = self.base_path / 'dsg' / f'{idx:06d}_{ts_sec}_{ts_nsec}.json'
            self.save_markerarray(dsg, dsg_fname)

        # append meta
        with self.meta_file.open('a') as f:
            f.write(f'{idx},{ts_sec},{ts_nsec},{rgb_fname.name},{"" if pcd_fname is None else pcd_fname.name},{"" if dsg is None else dsg_fname.name},{odom_fname.name}\n')

        # advance counter and clear some buffers optionally
        self.counter += 1
        # Optionally clear buffers to force new messages next frame:
        # self.buff = {k: None for k in self.buff}

        self.get_logger().info(f"Saved frame {idx} ts={ts_sec}.{ts_nsec}")

    def save_episode(self):
        # optionally consolidate episode metadata
        self.get_logger().info("Episode finished.")

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