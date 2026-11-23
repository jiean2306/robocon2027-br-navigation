#!/usr/bin/env python3
"""
Interactive On-Screen Virtual Joystick + WASD + Arrow Keys + Wheel-Lifting Climb Controller GUI
for ABU Robocon 2027 Builder Robot (BR v7).

Controls
--------
* Virtual 2D Joystick Pad: Click & drag with mouse for holonomic (vx, vy) mecanum motion.
* Keyboard (when GUI window is focused):
    - W / Up Arrow    : Drive Forward (+vx)
    - S / Down Arrow  : Drive Backward (-vx)
    - A / Left Arrow  : Strafe Left (+vy, holonomic mecanum)
    - D / Right Arrow : Strafe Right (-vy, holonomic mecanum)
    - Q / E           : Rotate CCW (+wz) / CW (-wz)
    - Space           : STOP all wheels
    - L               : Trigger 3-Stage Wheel-Lifting Climb UP (Ground -> L1 -> L2)
    - K               : Trigger 3-Stage Wheel-Lifting Descend (L2 -> L1 -> Ground)
    - 0 / 1 / 2 / R   : Jump/Climb to Ground Start (0), Level 1 (1), Level 2 (2), L1 Retry (R)
    - M               : Toggle MANUAL Mode <-> AUTONOMOUS A* Path-Following Mode
"""

from __future__ import annotations

import math
import sys
from typing import Set

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class TeleopGuiNode(Node):
    """ROS 2 publisher/subscriber backend for the Tkinter Joystick & WASD/Arrow GUI."""

    def __init__(self) -> None:
        super().__init__('br_teleop_gui')
        self.declare_parameter('max_linear_speed_mps', 0.90)
        self.declare_parameter('max_angular_speed_radps', 1.60)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.climb_pub = self.create_publisher(String, '/br_climb_command', 10)
        self.mission_pub = self.create_publisher(String, '/br_mission_command', 10)
        self.status_sub = self.create_subscription(
            String, '/br_climb_status', self._status_cb, 10
        )
        self.mission_status_sub = self.create_subscription(
            String, '/br_mission_status', self._mission_status_cb, 10
        )
        self.latest_status = (
            'Mode: MANUAL | Zone: GROUND START ZONE (-4.35, -5.15, z=0.00m) | Wheel-Lift: IDLE'
        )
        self.latest_mission_status = 'Mission: IDLE (Press G or click Run Full Match Demo)'
        self.auto_mode = False

    def _status_cb(self, msg: String) -> None:
        self.latest_status = msg.data
        self.auto_mode = 'Mode: AUTO' in msg.data

    def _mission_status_cb(self, msg: String) -> None:
        self.latest_mission_status = msg.data

    def publish_twist(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def send_climb_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.climb_pub.publish(msg)

    def send_mission_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.mission_pub.publish(msg)


def main(args=None) -> None:
    try:
        import tkinter as tk
    except Exception as exc:
        print(f'Tkinter GUI unavailable ({exc}); use `ros2 run br_nav_sandbox teleop_keyboard`.')
        return

    rclpy.init(args=args)
    node = TeleopGuiNode()

    try:
        root = tk.Tk()
    except Exception as exc:
        print(f'No display for Teleop GUI ({exc}); exiting cleanly.')
        node.destroy_node()
        rclpy.shutdown()
        return

    root.title('BR v7 Controller (Joystick / WASD / Full Match Demo)')
    root.geometry('445x615+20+320')
    root.configure(bg='#1e222a')

    max_v = float(node.get_parameter('max_linear_speed_mps').value)
    max_w = float(node.get_parameter('max_angular_speed_radps').value)

    pressed_keys: Set[str] = set()
    joy_state = {'active': False, 'vx': 0.0, 'vy': 0.0, 'wz': 0.0}

    # Header
    tk.Label(
        root,
        text='BR v7 Mecanum & Wheel-Lift Controller',
        font=('Helvetica', 12, 'bold'),
        fg='#61afef',
        bg='#1e222a',
        pady=4,
    ).pack()

    status_var = tk.StringVar(value=node.latest_status)
    status_lbl = tk.Label(
        root,
        textvariable=status_var,
        font=('Courier', 8, 'bold'),
        fg='#98c379',
        bg='#282c34',
        wraplength=410,
        justify='left',
        padx=6,
        pady=4,
        relief='groove',
    )
    status_lbl.pack(fill='x', padx=10, pady=3)

    # Virtual 2D Joystick Canvas
    canvas_size = 180
    center = canvas_size // 2
    max_r = 68
    canvas = tk.Canvas(
        root,
        width=canvas_size,
        height=canvas_size,
        bg='#282c34',
        highlightthickness=2,
        highlightbackground='#4b5263',
    )
    canvas.pack(pady=6)

    # Draw joystick rings and axes
    canvas.create_oval(
        center - max_r, center - max_r, center + max_r, center + max_r, outline='#5c6370', width=2
    )
    canvas.create_oval(
        center - max_r // 2,
        center - max_r // 2,
        center + max_r // 2,
        center + max_r // 2,
        outline='#3e4451',
        width=1,
    )
    canvas.create_line(center, 12, center, canvas_size - 12, fill='#3e4451', dash=(3, 3))
    canvas.create_line(12, center, canvas_size - 12, center, fill='#3e4451', dash=(3, 3))
    canvas.create_text(center, 14, text='FWD (+Vx / W / ↑)', fill='#abb2bf', font=('Helvetica', 8))
    canvas.create_text(
        center, canvas_size - 12, text='REV (-Vx / S / ↓)', fill='#abb2bf', font=('Helvetica', 8)
    )
    canvas.create_text(38, center - 10, text='LEFT\n(+Vy)', fill='#abb2bf', font=('Helvetica', 8))
    canvas.create_text(
        canvas_size - 38, center - 10, text='RIGHT\n(-Vy)', fill='#abb2bf', font=('Helvetica', 8)
    )

    knob_r = 20
    knob = canvas.create_oval(
        center - knob_r,
        center - knob_r,
        center + knob_r,
        center + knob_r,
        fill='#e06c75',
        outline='#ffffff',
        width=2,
    )

    vel_var = tk.StringVar(value='Vx: +0.00 m/s | Vy: +0.00 m/s | Wz: +0.00 rad/s')
    tk.Label(
        root, textvariable=vel_var, font=('Courier', 10, 'bold'), fg='#e5c07b', bg='#1e222a'
    ).pack(pady=2)

    def update_knob_position(dx: float, dy: float) -> None:
        canvas.coords(
            knob,
            center + dx - knob_r,
            center + dy - knob_r,
            center + dx + knob_r,
            center + dy + knob_r,
        )

    def on_joy_drag(event) -> None:
        node.send_climb_cmd('mode_manual')
        dx = float(event.x - center)
        dy = float(event.y - center)
        dist = math.hypot(dx, dy)
        if dist > max_r:
            dx = dx * max_r / dist
            dy = dy * max_r / dist
        update_knob_position(dx, dy)
        joy_state['active'] = True
        joy_state['vx'] = (-dy / max_r) * max_v
        joy_state['vy'] = (-dx / max_r) * max_v

    def on_joy_release(_event) -> None:
        joy_state['active'] = False
        joy_state['vx'] = 0.0
        joy_state['vy'] = 0.0
        update_knob_position(0.0, 0.0)
        node.publish_twist(0.0, 0.0, 0.0)

    canvas.bind('<ButtonPress-1>', on_joy_drag)
    canvas.bind('<B1-Motion>', on_joy_drag)
    canvas.bind('<ButtonRelease-1>', on_joy_release)

    # Rotation & Stop row
    rot_frame = tk.Frame(root, bg='#1e222a')
    rot_frame.pack(pady=4)

    def start_rot(wz_val: float) -> None:
        node.send_climb_cmd('mode_manual')
        joy_state['wz'] = wz_val

    def stop_all() -> None:
        pressed_keys.clear()
        joy_state['active'] = False
        joy_state['vx'] = joy_state['vy'] = joy_state['wz'] = 0.0
        update_knob_position(0.0, 0.0)
        node.publish_twist(0.0, 0.0, 0.0)

    btn_ccw = tk.Button(
        rot_frame,
        text='↺ Rotate CCW [Q]',
        bg='#3b4048',
        fg='white',
        width=15,
        command=lambda: start_rot(max_w * 0.7),
    )
    btn_ccw.grid(row=0, column=0, padx=4)

    btn_stop = tk.Button(
        rot_frame,
        text='■ STOP [Space]',
        bg='#e06c75',
        fg='white',
        font=('Helvetica', 10, 'bold'),
        width=14,
        command=stop_all,
    )
    btn_stop.grid(row=0, column=1, padx=4)

    btn_cw = tk.Button(
        rot_frame,
        text='↻ Rotate CW [E]',
        bg='#3b4048',
        fg='white',
        width=15,
        command=lambda: start_rot(-max_w * 0.7),
    )
    btn_cw.grid(row=0, column=2, padx=4)

    # Wheel-Lifting & Level-Transition Panel
    climb_box = tk.LabelFrame(
        root,
        text=' Fusion 360 Wheel-Lifting & Level Transitions (Ground <-> L1 <-> L2) ',
        fg='#61afef',
        bg='#1e222a',
        font=('Helvetica', 10, 'bold'),
        padx=8,
        pady=6,
    )
    climb_box.pack(fill='x', padx=12, pady=6)

    tk.Button(
        climb_box,
        text='▲ Wheel-Lift CLIMB UP (+Level) [Key: L]',
        bg='#98c379',
        fg='#1e222a',
        font=('Helvetica', 10, 'bold'),
        command=lambda: node.send_climb_cmd('climb_up'),
    ).pack(fill='x', pady=2)

    tk.Button(
        climb_box,
        text='▼ Wheel-Lift DESCEND (-Level) [Key: K]',
        bg='#d19a66',
        fg='#1e222a',
        font=('Helvetica', 10, 'bold'),
        command=lambda: node.send_climb_cmd('climb_down'),
    ).pack(fill='x', pady=2)

    lvl_row = tk.Frame(climb_box, bg='#1e222a')
    lvl_row.pack(fill='x', pady=3)
    for col_idx, (label, cmd_str) in enumerate(
        (
            ('[0] Ground Start', 'ground'),
            ('[1] Level 1 (+0.6m)', 'level_1'),
            ('[R] L1 Retry Zone', 'l1_retry'),
            ('[2] Level 2 (+0.9m)', 'level_2'),
        )
    ):
        tk.Button(
            lvl_row,
            text=label,
            bg='#3b4048',
            fg='white',
            font=('Helvetica', 8, 'bold'),
            command=lambda c=cmd_str: node.send_climb_cmd(c),
        ).grid(row=0, column=col_idx, padx=2, sticky='ew')
        lvl_row.grid_columnconfigure(col_idx, weight=1)

    # Mode & Dynamic Enemy Robot Controls
    mode_frame = tk.Frame(root, bg='#1e222a')
    mode_frame.pack(fill='x', padx=12, pady=4)
    tk.Button(
        mode_frame,
        text='🎮 Switch to MANUAL (WASD / Arrows / Joystick)',
        bg='#56b6c2',
        fg='#1e222a',
        font=('Helvetica', 9, 'bold'),
        command=lambda: node.send_climb_cmd('mode_manual'),
    ).pack(fill='x', pady=2)
    tk.Button(
        mode_frame,
        text='🤖 Switch to AUTONOMOUS (Follow RViz 2D Goal Pose A* Path)',
        bg='#c678dd',
        fg='#1e222a',
        font=('Helvetica', 9, 'bold'),
        command=lambda: node.send_climb_cmd('mode_auto'),
    ).pack(fill='x', pady=2)

    enemy_row = tk.Frame(mode_frame, bg='#1e222a')
    enemy_row.pack(fill='x', pady=2)
    tk.Button(
        enemy_row,
        text='🤖 Toggle OUR TR / ENEMY BR+TR [O]',
        bg='#3a6be8',
        fg='white',
        font=('Helvetica', 8, 'bold'),
        command=lambda: node.send_climb_cmd('enemy_toggle'),
    ).grid(row=0, column=0, padx=2, sticky='ew')
    tk.Button(
        enemy_row,
        text='⏸ Pause / Resume Match Robots [P]',
        bg='#4b5263',
        fg='white',
        font=('Helvetica', 8, 'bold'),
        command=lambda: node.send_climb_cmd('enemy_pause'),
    ).grid(row=0, column=1, padx=2, sticky='ew')
    enemy_row.grid_columnconfigure(0, weight=1)
    enemy_row.grid_columnconfigure(1, weight=1)

    # Full ABU Robocon 2027 Match Gameplay Demo Controls
    mission_row = tk.Frame(mode_frame, bg='#1e222a')
    mission_row.pack(fill='x', pady=3)
    tk.Button(
        mission_row,
        text='🏆 Run Full Match Demo (TR+BR+2 Towers+Mustika) [G]',
        bg='#e5c07b',
        fg='#1e222a',
        font=('Helvetica', 8, 'bold'),
        command=lambda: node.send_mission_cmd('start'),
    ).grid(row=0, column=0, padx=2, sticky='ew')
    tk.Button(
        mission_row,
        text='■ Stop Demo',
        bg='#e06c75',
        fg='white',
        font=('Helvetica', 8, 'bold'),
        command=lambda: node.send_mission_cmd('stop'),
    ).grid(row=0, column=1, padx=2, sticky='ew')
    mission_row.grid_columnconfigure(0, weight=3)
    mission_row.grid_columnconfigure(1, weight=1)

    mission_var = tk.StringVar(value=node.latest_mission_status)
    tk.Label(
        root,
        textvariable=mission_var,
        font=('Courier', 8, 'bold'),
        fg='#e5c07b',
        bg='#282c34',
        wraplength=420,
        justify='left',
        padx=6,
        pady=3,
        relief='groove',
    ).pack(fill='x', padx=10, pady=2)

    tk.Label(
        root,
        text='Keys: WASD/Arrows | Q/E Turn | L/K Lift | 0/1/2/R Levels | G Match Demo | O/P Enemy',
        font=('Helvetica', 8),
        fg='#abb2bf',
        bg='#1e222a',
    ).pack(pady=2)

    tk.Label(
        root,
        text='Map Legend: Cyan=Inscribed(0.35m) | Red->Yellow=Decay(0.80m) | Magenta=Dynamic | Green=A*',
        font=('Helvetica', 7, 'bold'),
        fg='#56b6c2',
        bg='#282c34',
        padx=4,
        pady=3,
    ).pack(fill='x', padx=10, pady=2)

    # Keyboard event handlers (WASD + Arrow Keys + Q/E + Space + L/K/0/1/2/R/M)
    def on_key_press(event) -> None:
        k = event.keysym.lower()
        if k == 'space':
            stop_all()
            return
        if k == 'l':
            node.send_climb_cmd('climb_up')
            return
        if k == 'k':
            node.send_climb_cmd('climb_down')
            return
        if k == '0':
            node.send_climb_cmd('ground')
            return
        if k == '1':
            node.send_climb_cmd('level_1')
            return
        if k == '2':
            node.send_climb_cmd('level_2')
            return
        if k == 'r':
            node.send_climb_cmd('l1_retry')
            return
        if k == 'm':
            node.send_climb_cmd('mode_manual' if node.auto_mode else 'mode_auto')
            return
        if k == 'o':
            node.send_climb_cmd('enemy_toggle')
            return
        if k == 'p':
            node.send_climb_cmd('enemy_pause')
            return
        if k == 'g':
            node.send_mission_cmd('toggle')
            return
        if k in ('w', 's', 'a', 'd', 'up', 'down', 'left', 'right', 'q', 'e'):
            node.send_climb_cmd('mode_manual')
            pressed_keys.add(k)

    def on_key_release(event) -> None:
        k = event.keysym.lower()
        pressed_keys.discard(k)
        if not pressed_keys and not joy_state['active']:
            joy_state['wz'] = 0.0
            update_knob_position(0.0, 0.0)
            node.publish_twist(0.0, 0.0, 0.0)

    root.bind('<KeyPress>', on_key_press)
    root.bind('<KeyRelease>', on_key_release)

    def periodic_tick() -> None:
        rclpy.spin_once(node, timeout_sec=0.0)
        status_var.set(node.latest_status)
        mission_var.set(node.latest_mission_status)

        vx = joy_state['vx']
        vy = joy_state['vy']
        wz = joy_state['wz']

        if pressed_keys:
            vx = 0.0
            vy = 0.0
            wz = 0.0
            if 'w' in pressed_keys or 'up' in pressed_keys:
                vx += max_v * 0.8
            if 's' in pressed_keys or 'down' in pressed_keys:
                vx -= max_v * 0.8
            if 'a' in pressed_keys or 'left' in pressed_keys:
                vy += max_v * 0.8
            if 'd' in pressed_keys or 'right' in pressed_keys:
                vy -= max_v * 0.8
            if 'q' in pressed_keys:
                wz += max_w * 0.75
            if 'e' in pressed_keys:
                wz -= max_w * 0.75
            update_knob_position((-vy / max_v) * max_r, (-vx / max_v) * max_r)

        if joy_state['active'] or pressed_keys or abs(wz) > 1e-4:
            node.publish_twist(vx, vy, wz)

        vel_var.set(f'Vx: {vx:+.2f} m/s | Vy: {vy:+.2f} m/s | Wz: {wz:+.2f} rad/s')
        root.after(50, periodic_tick)

    root.after(50, periodic_tick)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
