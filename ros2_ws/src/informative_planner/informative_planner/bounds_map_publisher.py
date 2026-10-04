"""Publica um OccupancyGrid com a zona segura do planejador para o Nav2.

Dentro de map_bounds (+margin) as células são livres (0); fora, letais (100). A static_layer dos
costmaps (nav2_no_map.yaml) lê este grid em /bounds_map, então o planejador global e o DWB
nunca levam o robô para fora da área de teste. O grid cobre outer_margin metros além dos bounds,
pois a static_layer não toca células fora da extensão do mapa.
"""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class BoundsMapPublisher(Node):
    def __init__(self):
        super().__init__('bounds_map_publisher')
        raw = str(self.declare_parameter('map_bounds', '').value)
        self.frame_id = str(self.declare_parameter('frame_id', 'map').value)
        self.margin = float(self.declare_parameter('margin', 0.6).value)
        self.outer_margin = float(self.declare_parameter('outer_margin', 10.0).value)
        self.resolution = float(self.declare_parameter('resolution', 0.1).value)
        self.topic = str(self.declare_parameter('topic', '/bounds_map').value)
        self.period_sec = float(self.declare_parameter('period_sec', 5.0).value)
        try:
            values = tuple(float(v) for v in raw.split(','))
        except ValueError:
            values = ()
        if len(values) != 4 or values[0] >= values[1] or values[2] >= values[3]:
            raise RuntimeError(f'map_bounds inválido: {raw!r} (esperado x_min,x_max,y_min,y_max)')
        self.bounds = values
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(OccupancyGrid, self.topic, qos)
        self.msg = self._build_grid()
        self.get_logger().info(
            f'bounds_map: frame={self.frame_id} livre em '
            f'[{values[0] - self.margin:.1f}, {values[1] + self.margin:.1f}] x '
            f'[{values[2] - self.margin:.1f}, {values[3] + self.margin:.1f}] m, '
            f'letal fora (até {self.outer_margin:.0f} m além), res={self.resolution} m -> {self.topic}'
        )
        self._publish()
        self.create_timer(self.period_sec, self._publish)

    def _build_grid(self):
        x_min, x_max, y_min, y_max = self.bounds
        res = self.resolution
        ox = x_min - self.margin - self.outer_margin
        oy = y_min - self.margin - self.outer_margin
        width = int(round((x_max - x_min + 2 * (self.margin + self.outer_margin)) / res))
        height = int(round((y_max - y_min + 2 * (self.margin + self.outer_margin)) / res))
        free_x0 = x_min - self.margin
        free_x1 = x_max + self.margin
        free_y0 = y_min - self.margin
        free_y1 = y_max + self.margin
        data = [100] * (width * height)
        for j in range(height):
            cy = oy + (j + 0.5) * res
            if not (free_y0 <= cy <= free_y1):
                continue
            row = j * width
            for i in range(width):
                cx = ox + (i + 0.5) * res
                if free_x0 <= cx <= free_x1:
                    data[row + i] = 0
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = res
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = ox
        msg.info.origin.position.y = oy
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        return msg

    def _publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.info.map_load_time = self.msg.header.stamp
        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoundsMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
