# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Vaibhav (VaibhavAero108)
"""AGASTYA control pad: a small window that turns held keys into manual commands.

Keys (window must have focus)
    W / S        forward / backward          A / D   strafe left / right
    Q / E        yaw left / right            R / F   climb / descend (also Up / Down arrows)
    SPACE        stop all motion (hold position)
    1 auto   2 air   3 underwater   0 off (motors stop)
    H            hold 0.35 m depth           C       cancel depth hold

Publishes geometry_msgs/Twist on /agastya/cmd_vel at 20 Hz. The vehicle-side
AgastyaDynamics system clamps the request to the air or water envelope.
"""
import json
import threading
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64, String

KEYMAP = {
    'w': 'fwd', 's': 'back', 'a': 'left', 'd': 'right',
    'q': 'yaw_l', 'e': 'yaw_r', 'r': 'up', 'f': 'down',
    'Up': 'up', 'Down': 'down', 'Left': 'yaw_l', 'Right': 'yaw_r',
}
RELEASE_DEBOUNCE_MS = 60  # X11 auto-repeat sends release/press pairs


class PadNode(Node):
    def __init__(self):
        super().__init__('agastya_control_pad')
        self.cmd_pub = self.create_publisher(Twist, '/agastya/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/agastya/mode', 10)
        self.depth_pub = self.create_publisher(Float64, '/agastya/depth/setpoint', 10)
        self.status = {}
        self.create_subscription(String, '/agastya/status', self.on_status, 10)

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            pass


class ControlPad:
    def __init__(self, node: PadNode):
        self.node = node
        self.held = set()
        self.pending_release = {}
        self.root = tk.Tk()
        self.root.title('AGASTYA control pad')
        self.root.geometry('430x560')
        self.root.configure(bg='#1d2126')
        self.speed = tk.DoubleVar(value=1.5)
        self.climb = tk.DoubleVar(value=0.8)
        self._build()
        self.root.bind('<KeyPress>', self.on_press)
        self.root.bind('<KeyRelease>', self.on_release)
        self.root.bind('<FocusOut>', lambda _e: self.held.clear())
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(50, self.tick)

    # ------------------------------------------------------------------ UI
    def _build(self):
        fg, bg, acc = '#e8eaed', '#1d2126', '#ff8c1a'
        font = ('DejaVu Sans Mono', 10)
        tk.Label(self.root, text='AGASTYA', fg=acc, bg=bg, font=('DejaVu Sans', 18, 'bold')).pack(pady=(8, 0))
        tk.Label(self.root, text='click here, then fly with the keyboard', fg='#9aa0a6', bg=bg).pack()

        self.status_var = tk.StringVar(value='waiting for /agastya/status ...')
        tk.Label(self.root, textvariable=self.status_var, fg=fg, bg='#262b31', font=font, justify='left',
                 anchor='w', width=50, height=8, padx=8).pack(padx=10, pady=8, fill='x')

        modes = tk.Frame(self.root, bg=bg)
        modes.pack(pady=2)
        for label, mode in (('1 AUTO', 'auto'), ('2 AIR', 'air'), ('3 WATER', 'underwater'), ('0 OFF', 'off')):
            tk.Button(modes, text=label, width=8, command=lambda m=mode: self.set_mode(m),
                      takefocus=0).pack(side='left', padx=3)
        depth = tk.Frame(self.root, bg=bg)
        depth.pack(pady=4)
        tk.Button(depth, text='H  hold 0.35 m depth', command=lambda: self.set_depth(0.35), takefocus=0).pack(side='left', padx=3)
        tk.Button(depth, text='C  cancel depth hold', command=lambda: self.set_depth(0.0), takefocus=0).pack(side='left', padx=3)

        sliders = tk.Frame(self.root, bg=bg)
        sliders.pack(pady=4, fill='x', padx=12)
        tk.Scale(sliders, label='horizontal speed request [m/s]', variable=self.speed, from_=0.1, to=3.0,
                 resolution=0.1, orient='horizontal', bg=bg, fg=fg, highlightthickness=0, takefocus=0).pack(fill='x')
        tk.Scale(sliders, label='vertical speed request [m/s]', variable=self.climb, from_=0.1, to=1.5,
                 resolution=0.1, orient='horizontal', bg=bg, fg=fg, highlightthickness=0, takefocus=0).pack(fill='x')

        self.keys_var = tk.StringVar(value='')
        tk.Label(self.root, textvariable=self.keys_var, fg=acc, bg=bg, font=font).pack()
        help_text = ('W/S fwd/back   A/D strafe   Q/E yaw\n'
                     'R/F or Up/Down climb/descend   SPACE stop\n'
                     '1 auto  2 air  3 underwater  0 off\n'
                     'H hold 0.35 m depth   C cancel depth hold\n'
                     'Start: hold R to take off from the pad.')
        tk.Label(self.root, text=help_text, fg='#9aa0a6', bg=bg, font=font, justify='left').pack(pady=6)

    # ------------------------------------------------------------- actions
    def set_mode(self, mode):
        self.node.mode_pub.publish(String(data=mode))

    def set_depth(self, depth):
        self.node.depth_pub.publish(Float64(data=float(depth)))

    def on_press(self, event):
        key = event.keysym if event.keysym in KEYMAP else event.keysym.lower()
        if key in self.pending_release:
            self.root.after_cancel(self.pending_release.pop(key))
        if key in KEYMAP:
            self.held.add(key)
        elif key == 'space':
            self.held.clear()
            self.idle_ticks = 0  # send a few explicit zero commands
        elif key in ('1', 'KP_1'):
            self.set_mode('auto')
        elif key in ('2', 'KP_2'):
            self.set_mode('air')
        elif key in ('3', 'KP_3'):
            self.set_mode('underwater')
        elif key in ('0', 'KP_0'):
            self.set_mode('off')
        elif key == 'h':
            self.set_depth(0.35)
        elif key == 'c':
            self.set_depth(0.0)

    def on_release(self, event):
        key = event.keysym if event.keysym in KEYMAP else event.keysym.lower()
        if key in KEYMAP:
            self.pending_release[key] = self.root.after(RELEASE_DEBOUNCE_MS, self._release, key)

    def _release(self, key):
        self.pending_release.pop(key, None)
        self.held.discard(key)

    def axis(self, positive, negative):
        held = {KEYMAP[k] for k in self.held}
        return (1.0 if positive in held else 0.0) - (1.0 if negative in held else 0.0)

    def tick(self):
        twist = Twist()
        twist.linear.x = self.speed.get() * self.axis('fwd', 'back')
        twist.linear.y = self.speed.get() * self.axis('left', 'right')
        twist.linear.z = self.climb.get() * self.axis('up', 'down')
        twist.angular.z = 1.0 * self.axis('yaw_l', 'yaw_r')
        # Publish only while flying with the pad (plus a short burst of zeros
        # after release) so other command sources can share /agastya/cmd_vel.
        # The vehicle holds position when commands stop.
        if self.held:
            self.idle_ticks = 0
        else:
            self.idle_ticks = getattr(self, 'idle_ticks', 99) + 1
        if self.idle_ticks <= 6:
            self.node.cmd_pub.publish(twist)
        self.keys_var.set('keys: ' + (' '.join(sorted(self.held)) or '-'))

        s = self.node.status
        if s:
            t = s.get('thrust', [0, 0, 0, 0])
            self.status_var.set(
                f"mode   {s.get('mode', '?'):<10} state  {s.get('state', '?')}\n"
                f"domain {s.get('domain', '?'):<10} submerged {s.get('submerged', 0):.2f}\n"
                f"pos    x {s.get('x', 0):6.2f}  y {s.get('y', 0):6.2f}  z {s.get('z', 0):6.2f}\n"
                f"depth  {s.get('depth', 0):.2f} m   speed {s.get('speed', 0):.2f} m/s\n"
                f"depth hold {'ON ' + format(s.get('depth_target', 0), '.2f') + ' m' if s.get('depth_hold') else 'off'}\n"
                f"rotor thrust N  {t[0]:4.1f} {t[1]:4.1f} {t[2]:4.1f} {t[3]:4.1f}\n"
                f"weight {s.get('weight', 0):.1f} N  buoyancy {s.get('buoyancy', 0):.1f} N\n"
                f"sim time {s.get('sim_time', 0):.1f} s")
        self.root.after(50, self.tick)

    def close(self):
        self.node.cmd_pub.publish(Twist())
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = PadNode()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        ControlPad(node).run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
