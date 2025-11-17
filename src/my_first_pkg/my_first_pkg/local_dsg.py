#!/usr/bin/env python3
"""
local_dsg.py — расширённый локальный scene-graph визуализатор.

Функционал:
- публикует MarkerArray с кубами (Marker.CUBE) и lifetime
- публикует TEXT_VIEW_FACING для надписей
- публикует LINE_STRIP для трека объектов (история позиций)
- умеет удалять маркеры (action=DELETE)
- интеграция с TF: чтение позиций по списку frame_id'ов
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped

import math
import time
from collections import defaultdict, deque


class LocalDSG(Node):
    def __init__(self):
        super().__init__('local_dsg')
        self.pub = self.create_publisher(MarkerArray, 'dsg_markers', 10)
        self.timer = self.create_timer(0.5, self.timer_cb)  # 2 Hz
        self.get_logger().info("LocalDSG started")

        # TF buffer & listener (для получения позиций по именам кадров)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Список имен фреймов, которые мы хотим визуализировать (можно редактировать)
        # Пример: если в системе публикуются кадры 'object_1', 'object_2', ...
        self.frame_ids_to_visualize = ['object_1', 'object_2']  # можно оставить пустым и генерировать локально

        # Хранилище для истории позиций (для LINE_STRIP): ограничим длину истории
        self.track_history = defaultdict(lambda: deque(maxlen=100))

        # Счётчик для генерации временных локальных объектов (если TF не задан)
        self.local_counter = 0

        # Параметры отображения
        self.lifetime_seconds = 20  # lifetime для маркеров
        self.track_enabled = True
        self.text_enabled = True

    # ---- вспомогательные функции для создания маркеров ----
    def make_cube_marker(self, ns: str, idx: int, position, scale=1.0, frame_id='world') -> Marker:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = idx
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(position[0])
        m.pose.position.y = float(position[1])
        m.pose.position.z = float(position[2])
        m.pose.orientation.w = 1.0
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        # color (cycle)
        color_cycle = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 1.0, 0.0)]
        r, g, b = color_cycle[idx % len(color_cycle)]
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.9
        # lifetime
        m.lifetime = Duration(sec=self.lifetime_seconds, nanosec=0)
        return m

    def make_text_marker(self, ns: str, idx: int, text: str, position, frame_id='world') -> Marker:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns + "_text"
        m.id = idx
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        # position a bit above the object
        m.pose.position.x = float(position[0])
        m.pose.position.y = float(position[1])
        m.pose.position.z = float(position[2]) + 1.2
        m.pose.orientation.w = 1.0
        m.scale.z = 0.6  # text height
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 0.95
        m.text = text
        m.lifetime = Duration(sec=self.lifetime_seconds, nanosec=0)
        return m

    def make_line_strip(self, ns: str, idx: int, points, frame_id='world') -> Marker:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns + "_track"
        m.id = idx
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.06  # line width
        # set color (white-ish)
        m.color.r = 1.0
        m.color.g = 0.6
        m.color.b = 0.0
        m.color.a = 0.9
        m.points = []
        for p in points:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = float(p[2])
            m.points.append(pt)
        m.lifetime = Duration(sec=self.lifetime_seconds, nanosec=0)
        return m

    # ---- удаление маркера по id (publish DELETE action) ----
    def make_delete_marker(self, ns: str, idx: int, frame_id='world') -> Marker:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = idx
        m.action = Marker.DELETE
        return m

    # ---- попытка получить позицию из TF; возвращает (x,y,z) или None ----
    def get_position_from_tf(self, frame_id: str, reference_frame: str = 'world'):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(reference_frame, frame_id, rclpy.time.Time())
            t = trans.transform.translation
            return (t.x, t.y, t.z)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # TF not available
            return None

    # ---- основной таймер-коллбек ----
    def timer_cb(self):
        # создаём контейнер для всех маркеров
        ma = MarkerArray()

        # текущее время в секундах (для движения объектов)
        t = self.get_clock().now().seconds_nanoseconds()[0] + \
            self.get_clock().now().seconds_nanoseconds()[1] * 1e-9

        # создаём 3 движущихся объекта for i in range(3):
        #             x = math.sin(0.5 * t + i) * (1.5 + 0.3 * i)
        #             y = math.cos(0.4 * t + i * 0.7) * (0.5 + 0.2 * i)
        #             z = 0.5
        #             p = (x, y, z)

        for i in range(30):
            u = 0.4 * t + i * 0.25
            v = 0.6 * t + i * 0.15
            R = 2.0  # Большой радиус тора
            r = 0.7  # Малый радиус тора

            x = (R + r * math.cos(v)) * math.cos(u)
            y = (R + r * math.cos(v)) * math.sin(u)
            z = 0.5 + r * math.sin(v) * 1.2
            p = (x, y, z)

            # куб
            ma.markers.append(
                self.make_cube_marker(
                    ns='local',
                    idx=i,
                    position=p,
                    frame_id='world'
                )
            )

            # текст, если включён
            if self.text_enabled:
                ma.markers.append(
                    self.make_text_marker(
                        ns='local',
                        idx=i,
                        text=f"obj_{i}",
                        position=p,
                        frame_id='world'
                    )
                )

            # трек, если включён
            if self.track_enabled:
                key = f"local_{i}"
                self.track_history[key].append((p[0], p[1], p[2] + 0.05))

                ma.markers.append(
                    self.make_line_strip(
                        ns='local',
                        idx=i,
                        points=list(self.track_history[key]),
                        frame_id='world'
                    )
                )

        # публикуем все маркеры
        self.pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = LocalDSG()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
