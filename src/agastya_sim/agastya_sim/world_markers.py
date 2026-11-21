# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Vaibhav (VaibhavAero108)
"""Publishes the pool, waterline, deck and landing pad as RViz markers (frame: world)."""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

# Must match worlds/agastya_dual_domain.sdf
OBJECTS = [
    # name, type, (x, y, z), (sx, sy, sz), (r, g, b, a)
    ('water_volume', Marker.CUBE, (0, 0, -0.6), (8.0, 6.0, 1.2), (0.05, 0.35, 0.65, 0.25)),
    ('waterline', Marker.CUBE, (0, 0, -0.001), (8.0, 6.0, 0.002), (0.1, 0.5, 0.9, 0.45)),
    ('pool_floor', Marker.CUBE, (0, 0, -1.3), (8.4, 6.4, 0.2), (0.35, 0.65, 0.75, 1.0)),
    ('deck_east', Marker.CUBE, (6.2, 0, 0.0), (4.0, 10.0, 0.2), (0.72, 0.7, 0.66, 1.0)),
    ('deck_west', Marker.CUBE, (-6.2, 0, 0.0), (4.0, 10.0, 0.2), (0.72, 0.7, 0.66, 1.0)),
    ('deck_north', Marker.CUBE, (0, 4.6, 0.0), (8.4, 3.0, 0.2), (0.72, 0.7, 0.66, 1.0)),
    ('deck_south', Marker.CUBE, (0, -4.6, 0.0), (8.4, 3.0, 0.2), (0.72, 0.7, 0.66, 1.0)),
    ('landing_pad', Marker.CUBE, (5.5, 0, 0.115), (1.2, 1.2, 0.03), (0.95, 0.55, 0.05, 1.0)),
    ('gate_left', Marker.CYLINDER, (-1.0, 0.6, -0.64), (0.08, 0.08, 1.12), (0.95, 0.2, 0.1, 1.0)),
    ('gate_right', Marker.CYLINDER, (-1.0, -0.6, -0.64), (0.08, 0.08, 1.12), (0.95, 0.2, 0.1, 1.0)),
    ('gate_bar', Marker.CUBE, (-1.0, 0, -0.08), (0.08, 1.28, 0.08), (0.95, 0.8, 0.1, 1.0)),
    ('tether_reel', Marker.CYLINDER, (6.6, 1.6, 0.22), (0.2, 0.2, 0.2), (1.0, 0.8, 0.1, 1.0)),
]


class WorldMarkers(Node):
    def __init__(self):
        super().__init__('agastya_world_markers')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, '/agastya/world_markers', qos)
        self.publish()
        self.create_timer(2.0, self.publish)

    def publish(self):
        array = MarkerArray()
        for i, (name, kind, pos, scale, color) in enumerate(OBJECTS):
            m = Marker()
            m.header.frame_id = 'world'
            m.ns = name
            m.id = i
            m.type = kind
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = map(float, scale)
            m.color.r, m.color.g, m.color.b, m.color.a = map(float, color)
            m.frame_locked = True
            array.markers.append(m)
        self.pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = WorldMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
