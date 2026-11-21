# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Vaibhav (VaibhavAero108)
"""Command/video link simulation for AGASTYA.

* Republishes the Gazebo camera (/agastya/fpv/image_raw) as the operator's
  FPV feed (/agastya/fpv/image), degraded according to the link quality the
  AgastyaDynamics system reports in /agastya/status:
    - fibre tether: clean picture (unless the tether breaks),
    - 5.8 GHz radio: noise, tearing and dropped frames as the antenna nears
      the surface, analog "snow" once it is under water.
* Draws the fibre tether (shore reel -> vehicle) in RViz (/agastya/tether_marker)
  and, when the Gazebo GUI is running, in the Gazebo 3D view.
"""
import json
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker

try:  # Gazebo transport Python bindings (shipped with the Jazzy gz vendor packages)
    from gz.msgs10.boolean_pb2 import Boolean as GzBoolean
    from gz.msgs10.marker_pb2 import Marker as GzMarker
    from gz.transport13 import Node as GzNode
except ImportError:  # pragma: no cover - visualisation is optional
    GzNode = None


class LinkSim(Node):
    def __init__(self):
        super().__init__('agastya_link_sim')
        self.quality = 1.0
        self.tether = []
        self.link_mode = 'fibre'
        self.rng = np.random.default_rng()
        # reliable publisher: compatible with both reliable and best-effort subscribers
        self.pub_img = self.create_publisher(Image, '/agastya/fpv/image', 2)
        self.pub_tether = self.create_publisher(Marker, '/agastya/tether_marker', 5)
        self.create_subscription(Image, '/agastya/fpv/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(String, '/agastya/status', self.on_status, 10)
        self.create_timer(0.2, self.publish_tether)
        self._gz_lock = threading.Lock()
        self._gz_points = None
        if GzNode is not None:
            threading.Thread(target=self._gz_marker_loop, daemon=True).start()

    # ------------------------------------------------------------------ status
    def on_status(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        self.quality = float(s.get('video_quality', 1.0))
        self.tether = s.get('tether', []) or []
        self.link_mode = s.get('link_mode', 'fibre')
        with self._gz_lock:
            self._gz_points = list(self.tether)

    # ------------------------------------------------------------------- video
    def on_image(self, msg):
        q = self.quality
        if q >= 0.99 or msg.encoding not in ('rgb8', 'bgr8'):
            self.pub_img.publish(msg)
            return
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)[:, :w * 3].reshape(h, w, 3)
        if q <= 0.02 or self.rng.random() < (1.0 - q) * 0.35:
            snow = self.rng.integers(0, 256, size=(h // 2, w // 2, 1), dtype=np.uint8)
            out = np.repeat(np.repeat(snow, 2, axis=0), 2, axis=1).repeat(3, axis=2)
        else:
            noise = self.rng.normal(0.0, 110.0 * (1.0 - q), size=(h // 4, w // 4, 1))
            noise = np.repeat(np.repeat(noise, 4, axis=0), 4, axis=1)
            out = np.clip(img.astype(np.float32) * (0.35 + 0.65 * q) + noise, 0, 255).astype(np.uint8)
            for _ in range(int((1.0 - q) * 12)):  # horizontal tearing bands
                y0 = int(self.rng.integers(0, h - 8))
                y1 = min(h, y0 + int(self.rng.integers(4, 30)))
                out[y0:y1] = np.roll(out[y0:y1], int(self.rng.integers(-60, 60)), axis=1)
        degraded = Image()
        degraded.header = msg.header
        degraded.height, degraded.width = h, w
        degraded.encoding = msg.encoding
        degraded.is_bigendian = msg.is_bigendian
        degraded.step = w * 3
        degraded.data = out.tobytes()
        self.pub_img.publish(degraded)

    # ------------------------------------------------------------------ tether
    def publish_tether(self):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'tether'
        m.id = 0
        if len(self.tether) < 2:
            m.action = Marker.DELETE
        else:
            m.action = Marker.ADD
            m.type = Marker.LINE_STRIP
            m.scale.x = 0.012
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.1, 1.0
            m.pose.orientation.w = 1.0
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in self.tether]
        self.pub_tether.publish(m)

    def _gz_marker_loop(self):
        node = GzNode()
        backoff = 0.0
        had_line = False
        while rclpy.ok():
            time.sleep(0.2)
            if time.monotonic() < backoff:
                continue
            with self._gz_lock:
                pts = self._gz_points
            if pts is None or (len(pts) < 2 and not had_line):
                continue
            msg = GzMarker()
            msg.ns = 'agastya_tether'
            msg.id = 1
            if len(pts) < 2:
                msg.action = GzMarker.DELETE_MARKER
            else:
                msg.action = GzMarker.ADD_MODIFY
                msg.type = GzMarker.LINE_STRIP
                msg.visibility = GzMarker.GUI
                for c in (msg.material.ambient, msg.material.diffuse, msg.material.emissive):
                    c.r, c.g, c.b, c.a = 1.0, 0.8, 0.1, 1.0
                for p in pts:
                    v = msg.point.add()
                    v.x, v.y, v.z = float(p[0]), float(p[1]), float(p[2])
            try:
                ok, _ = node.request('/marker', msg, GzMarker, GzBoolean, 300)
            except Exception:  # noqa: BLE001 - GUI not running / API mismatch
                ok = False
            if ok:
                had_line = len(pts) >= 2
            else:
                backoff = time.monotonic() + 5.0  # Gazebo GUI not available; retry later


def main(args=None):
    rclpy.init(args=args)
    node = LinkSim()
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
