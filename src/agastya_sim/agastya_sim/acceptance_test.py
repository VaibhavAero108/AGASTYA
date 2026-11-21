# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Vaibhav (VaibhavAero108)
"""Automated AGASTYA acceptance mission (run while the simulation is up).

    ros2 run agastya_sim acceptance_test

Flies: idle on pad -> FPV check -> take-off -> manual air moves -> fly over the
pool -> descend through the waterline -> 0.35 m depth hold -> swim -> surface
-> fly back -> land on the pad -> 'off' stop check. Prints PASS/FAIL per step.
"""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float64, String

PAD = (5.5, 0.0)
POOL_CENTRE = (1.5, 0.0)  # dive point; the swim then stays clear of the gate at x = -1


class Mission(Node):
    def __init__(self):
        super().__init__('agastya_acceptance_test',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.cmd = self.create_publisher(Twist, '/agastya/cmd_vel', 10)
        self.mode = self.create_publisher(String, '/agastya/mode', 10)
        self.depth = self.create_publisher(Float64, '/agastya/depth/setpoint', 10)
        self.status = None
        self.odom = None
        self.images = 0
        self.max_acc = 0.0
        self.track_acc = False
        self.create_subscription(String, '/agastya/status', self._status, 10)
        self.create_subscription(Odometry, '/agastya/odometry', self._odom, 10)
        self.create_subscription(Image, '/agastya/fpv/image', self._image, 5)
        self.create_subscription(Imu, '/agastya/imu', self._imu, 50)
        self.results = []

    # ------------------------------------------------------------ callbacks
    def _status(self, m):
        try:
            self.status = json.loads(m.data)
        except ValueError:
            pass

    def _odom(self, m):
        self.odom = m

    def _image(self, _m):
        self.images += 1

    def _imu(self, m):
        if self.track_acc:
            a = m.linear_acceleration
            self.max_acc = max(self.max_acc, math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z))

    # -------------------------------------------------------------- helpers
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def yaw(self):
        q = self.odom.pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def tilt(self):
        q = self.odom.pose.pose.orientation
        return math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y))))

    def pos(self):
        s = self.status
        return s['x'], s['y'], s['z']

    def send(self, vx=0.0, vy=0.0, vz=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z, t.angular.z = float(vx), float(vy), float(vz), float(wz)
        self.cmd.publish(t)

    def run(self, sim_seconds, command=None, until=None, monitor=None):
        """Publish command() at ~20 Hz until sim_seconds elapse or until() is true."""
        wall_start = time.monotonic()
        while rclpy.ok() and self.now() <= 0.0:  # wait for the first /clock message
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - wall_start > 60.0:
                self.get_logger().error('no /clock from the simulation')
                return False
        start = self.now()
        last_pub = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if time.monotonic() - last_pub > 0.05:
                last_pub = time.monotonic()
                args = command() if command else {}
                self.send(**args)
                if monitor and self.status and self.odom:
                    monitor()
            if until and self.status and self.odom and until():
                return True
            if self.now() - start >= sim_seconds:
                return until is None
            if time.monotonic() - wall_start > sim_seconds * 25 + 60:
                self.get_logger().error('wall-clock timeout (simulation stalled?)')
                return False
        return False

    def goto_cmd(self, target, z_target, gain=0.9, vmax=1.5):
        def command():
            x, y, z = self.pos()
            ex, ey = target[0] - x, target[1] - y
            c, s = math.cos(self.yaw()), math.sin(self.yaw())
            bx, by = c * ex + s * ey, -s * ex + c * ey
            n = math.hypot(bx, by)
            k = min(vmax, gain * n) / n if n > 1e-6 else 0.0
            return {'vx': bx * k, 'vy': by * k, 'vz': max(-0.8, min(0.8, 1.2 * (z_target - z)))}
        return command

    def check(self, name, ok, detail):
        self.results.append((name, ok, detail))
        self.get_logger().info(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")
        return ok

    # -------------------------------------------------------------- mission
    def mission(self):
        log = self.get_logger()
        log.info('waiting for /agastya/status, odometry and clock ...')
        if not self.run(60.0, until=lambda: True):
            self.check('startup', False, 'no status/odometry from simulation')
            return
        self.mode.publish(String(data='auto'))
        self.run(1.0)

        # 1. idle on the pad
        x0, y0, z0 = self.pos()
        self.run(10.0)
        x1, y1, z1 = self.pos()
        drift = math.dist((x0, y0, z0), (x1, y1, z1))
        self.check('1 idle 10 s on pad', drift < 0.03 and self.status['state'] == 'LANDED',
                   f'drift {drift:.3f} m, state {self.status["state"]}, z {z1:.3f}')

        # 2. FPV camera
        self.images = 0
        t0, w0 = self.now(), time.monotonic()
        self.run(3.0)
        sim_rate = self.images / max(1e-3, self.now() - t0)
        wall_rate = self.images / max(1e-3, time.monotonic() - w0)
        self.check('2 FPV image stream', self.images > 5,
                   f'{self.images} frames, {sim_rate:.1f} Hz sim, {wall_rate:.1f} Hz wall')

        # 3. take-off and hover
        self.run(2.0, lambda: {'vz': 0.8})
        self.run(2.0)
        hx, hy, hz = self.pos()
        max_tilt = [0.0]
        self.run(4.0, monitor=lambda: max_tilt.__setitem__(0, max(max_tilt[0], self.tilt())))
        ex, ey, ez = self.pos()
        hover_drift = math.dist((hx, hy, hz), (ex, ey, ez))
        self.check('3 take-off + hover hold', self.status['state'] == 'FLYING' and ez > 0.9 and hover_drift < 0.15,
                   f'z {ez:.2f} m, 4 s hover drift {hover_drift:.3f} m, max tilt {math.degrees(max_tilt[0]):.1f} deg')

        # 4. manual air response (body-frame forward / strafe / yaw)
        speeds = [0.0]
        mon = lambda: speeds.__setitem__(0, max(speeds[0], self.status['speed']))  # noqa: E731
        ax, ay, _ = self.pos()
        self.run(2.0, lambda: {'vx': 1.5}, monitor=mon)
        bx, by, _ = self.pos()
        fwd = math.hypot(bx - ax, by - ay)
        yaw0 = self.yaw()
        self.run(1.5, lambda: {'wz': 1.0}, monitor=mon)
        dyaw = math.atan2(math.sin(self.yaw() - yaw0), math.cos(self.yaw() - yaw0))
        self.run(1.5, lambda: {'wz': -1.0}, monitor=mon)
        self.run(1.5, lambda: {'vy': 1.0}, monitor=mon)
        self.run(2.0, monitor=mon)
        self.check('4 manual air control', fwd > 1.5 and dyaw > 0.8 and speeds[0] < 3.5,
                   f'forward {fwd:.2f} m in 2 s, yaw {math.degrees(dyaw):.0f} deg in 1.5 s, peak speed {speeds[0]:.2f} m/s')

        # fly over the pool
        ok = self.run(30.0, self.goto_cmd(POOL_CENTRE, 1.0),
                      until=lambda: math.dist(self.pos()[:2], POOL_CENTRE) < 0.2)
        self.run(2.0)
        self.check('  reach pool centre', ok, 'pos ({:.2f}, {:.2f}, {:.2f})'.format(*self.pos()))

        # 5. air -> water transition
        self.max_acc, self.track_acc = 0.0, True
        ok = self.run(30.0, lambda: {'vz': -0.6}, until=lambda: self.status['domain'] == 'UNDERWATER')
        self.run(2.0)
        self.track_acc = False
        self.check('5 air -> water transition', ok and self.max_acc < 30.0,
                   f'domain {self.status["domain"]}, depth {self.status["depth"]:.2f} m, '
                   f'peak IMU accel {self.max_acc:.1f} m/s^2')

        # 6. depth hold
        self.depth.publish(Float64(data=0.35))
        self.run(15.0)
        d = self.status['depth']
        err = [0.0]
        self.run(4.0, monitor=lambda: err.__setitem__(0, max(err[0], abs(self.status['depth'] - 0.35))))
        self.check('6 depth hold 0.35 m', abs(d - 0.35) < 0.08 and err[0] < 0.1,
                   f'depth {self.status["depth"]:.3f} m, max error over 4 s {err[0]:.3f} m')

        # 7. underwater drive
        sx, sy, _ = self.pos()
        derr = [0.0]
        self.run(8.0, lambda: {'vx': 0.4},
                 monitor=lambda: derr.__setitem__(0, max(derr[0], abs(self.status['depth'] - 0.35))))
        tx, ty, _ = self.pos()
        dist = math.hypot(tx - sx, ty - sy)
        self.run(3.0)
        self.check('7 underwater horizontal drive', dist > 0.6 and derr[0] < 0.2 and self.status['domain'] == 'UNDERWATER',
                   f'moved {dist:.2f} m in 8 s, max depth error {derr[0]:.3f} m')

        # 8. surface and take off from the water while still moving forward
        self.depth.publish(Float64(data=0.0))
        self.max_acc, self.track_acc = 0.0, True
        breach_tilt = [0.0]
        ok = self.run(30.0, lambda: {'vz': 0.8, 'vx': 0.3},
                      until=lambda: self.status['domain'] == 'AIR' and self.pos()[2] > 0.8,
                      monitor=lambda: breach_tilt.__setitem__(0, max(breach_tilt[0], self.tilt())))
        self.track_acc = False
        self.run(3.0)
        self.check('8 water -> air transition', ok and self.status['domain'] == 'AIR' and
                   self.status['state'] == 'FLYING' and breach_tilt[0] < math.radians(35),
                   f'domain {self.status["domain"]}, z {self.pos()[2]:.2f} m, max tilt during breach '
                   f'{math.degrees(breach_tilt[0]):.1f} deg, peak IMU accel {self.max_acc:.1f} m/s^2')

        # 9. return to pad and land
        ok_nav = self.run(40.0, self.goto_cmd(PAD, 1.0),
                          until=lambda: math.dist(self.pos()[:2], PAD) < 0.12)
        self.run(2.0, self.goto_cmd(PAD, 1.0))
        ok_land = self.run(25.0, lambda: dict(self.goto_cmd(PAD, -5.0)(), vz=-0.5),
                           until=lambda: self.status['state'] == 'LANDED')
        self.run(3.0)
        lx, ly, lz = self.pos()
        self.check('9 land on pad', ok_nav and ok_land and math.dist((lx, ly), PAD) < 0.4 and lz < 0.4,
                   f'state {self.status["state"]}, pos ({lx:.2f}, {ly:.2f}, {lz:.2f})')

        # 10. stop command
        self.run(1.5, lambda: {'vz': 0.8})
        self.mode.publish(String(data='off'))
        self.run(1.0)
        thrust = max(self.status['thrust'])
        self.run(3.0)
        self.check("10 'off' removes all rotor thrust", thrust < 0.05 and self.status['state'] == 'LANDED',
                   f'max rotor thrust {thrust:.3f} N after 1 s, state {self.status["state"]}')
        self.mode.publish(String(data='auto'))
        self.run(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    try:
        node.mission()
    except KeyboardInterrupt:
        pass
    passed = sum(1 for _, ok, _ in node.results if ok)
    print('\n================ AGASTYA acceptance summary ================')
    for name, ok, detail in node.results:
        print(f"{'PASS' if ok else 'FAIL'}  {name:34s} {detail}")
    print(f'{passed}/{len(node.results)} checks passed')
    node.send()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed == len(node.results) and node.results else 1)


if __name__ == '__main__':
    main()
