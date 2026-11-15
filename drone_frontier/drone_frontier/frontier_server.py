#!/usr/bin/env python3
"""frontier_server - the edge-finder (guidebook 2.7). It finds frontiers and publishes them; it sends no commands.

  in:   /map (nav_msgs/OccupancyGrid, Cartographer or map_server), TF map -> base_link (the drone)
  out:  /frontiers (drone_interfaces/FrontierArray, cheapest first) at rate_hz
        /frontiers/markers (RViz: a sphere per frontier, larger = bigger opening, text = rank)
        /frontiers/search_ms (std_msgs/Float32: time of the last search, for C3/C7)
Parameters: rate_hz 1.0, free_threshold 65, occupied_threshold 65, min_frontier_size 0.55, potential_scale 3.0,
gain_scale 1.0, map_frame map, base_frame base_link.
free_threshold: the book says 25, but Cartographer's free cells in our simulation sit at 40-49 (workstream A), and
the planner's cost map treats every observed value below 65 as free; the two must agree, so 65 (see C3).
"""
import time

import numpy as np
import rclpy
from drone_interfaces.msg import Frontier, FrontierArray
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .frontier_core import Params, search


class FrontierServer(Node):
    def __init__(self):
        super().__init__('frontier_server')
        P = self.declare_parameter
        self.rate = P('rate_hz', 1.0).value
        self.p = Params(free_threshold=P('free_threshold', 65).value,
                        occupied_threshold=P('occupied_threshold', 65).value,
                        min_frontier_size=P('min_frontier_size', 0.55).value,
                        potential_scale=P('potential_scale', 3.0).value,
                        gain_scale=P('gain_scale', 1.0).value)
        self.map_frame = P('map_frame', 'map').value
        self.base_frame = P('base_frame', 'base_link').value
        self.map = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.pub = self.create_publisher(FrontierArray, '/frontiers', 10)
        self.pub_m = self.create_publisher(MarkerArray, '/frontiers/markers', 10)
        self.pub_t = self.create_publisher(Float32, '/frontiers/search_ms', 10)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.create_timer(1.0 / self.rate, self.tick)
        self.warned = 0.0
        self.get_logger().info(f'frontier server: {self.p}')

    def on_map(self, m):
        self.map = m

    def robot(self):
        try:
            t = self.tf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception as e:  # noqa: BLE001
            if time.time() - self.warned > 5:
                self.get_logger().warning(f'no drone pose ({self.map_frame} -> {self.base_frame}): {e}')
                self.warned = time.time()
            return None

    def tick(self):
        m, xy = self.map, self.robot()
        if m is None or xy is None:
            return
        t0 = time.perf_counter()
        data = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        fr, _ = search(data, m.info.resolution, (m.info.origin.position.x, m.info.origin.position.y), xy, self.p)
        ms = (time.perf_counter() - t0) * 1000
        out = FrontierArray()
        out.header.frame_id = self.map_frame
        out.header.stamp = self.get_clock().now().to_msg()
        for f in fr:
            out.frontiers.append(Frontier(centroid=Point(x=f.centroid[0], y=f.centroid[1]),
                                          middle=Point(x=f.middle[0], y=f.middle[1]),
                                          nearest=Point(x=f.nearest[0], y=f.nearest[1]),
                                          size_m=float(f.size_m), distance_m=float(f.distance_m), cost=float(f.cost)))
        self.pub.publish(out)
        self.pub_t.publish(Float32(data=float(ms)))
        self.markers(out)

    def markers(self, out):
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for i, f in enumerate(out.frontiers):
            for kind in (Marker.SPHERE, Marker.TEXT_VIEW_FACING):
                mk = Marker()
                mk.header = out.header
                mk.ns, mk.id, mk.type = ('frontier' if kind == Marker.SPHERE else 'rank'), i, kind
                mk.pose.position = Point(x=f.centroid.x, y=f.centroid.y, z=0.1 if kind == Marker.SPHERE else 0.5)
                mk.pose.orientation.w = 1.0
                s = min(1.0, 0.15 + 0.2 * f.size_m)
                mk.scale.x = mk.scale.y = mk.scale.z = s if kind == Marker.SPHERE else 0.3
                mk.color.a, mk.color.r, mk.color.g, mk.color.b = 1.0, (0.1 if i else 1.0), (1.0 if i else 0.5), 0.1
                mk.text = str(i + 1)
                ma.markers.append(mk)
        self.pub_m.publish(ma)


def main():
    rclpy.init()
    n = FrontierServer()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
