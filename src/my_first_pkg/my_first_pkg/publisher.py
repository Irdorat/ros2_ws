import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello SUKA KAK TAK: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)  # ШАГ 1: Инициализация (БЕЗ 'with')
    node = MinimalPublisher()  # ШАГ 2: Создание объекта ноды

    try:
        rclpy.spin(node)  # ШАГ 3: Запуск основного цикла обработки
    except KeyboardInterrupt:
        pass  # Обработка прерывания (Ctrl+C)
    finally:
        node.destroy_node()  # ШАГ 4: Корректное уничтожение ноды
        rclpy.shutdown()  # ШАГ 5: Корректное завершение rclpy


if __name__ == '__main__':
    main()