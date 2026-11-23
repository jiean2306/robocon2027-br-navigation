#!/usr/bin/env python3
"""
Terminal Keyboard Controller (WASD + Arrow Keys + Wheel-Lifting Climb)
for ABU Robocon 2027 Builder Robot (BR v7).

Usage:
    ros2 run br_nav_sandbox teleop_keyboard
"""

from __future__ import annotations

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


BANNER = """
========================================================================
 ABU Robocon 2027 — BR v7 Terminal Keyboard & Wheel-Lift Controller
========================================================================
 Holonomic Mecanum Drive:
   W / Up Arrow    : Forward (+Vx)
   S / Down Arrow  : Backward (-Vx)
   A / Left Arrow  : Strafe Left (+Vy)
   D / Right Arrow : Strafe Right (-Vy)
   Q / E           : Rotate CCW (+Wz) / Rotate CW (-Wz)
   Space           : STOP (0, 0, 0)

 Wheel-Lifting Multi-Level Climber (Ground <-> L1 <-> L2):
   L               : Wheel-Lift CLIMB UP (Ground -> Level 1 -> Level 2)
   K               : Wheel-Lift DESCEND  (Level 2 -> Level 1 -> Ground)
   0               : Reset to Ground Start/Retry Zone (-4.35, -5.15, z=0.0m)
   1               : Climb to Level 1 (+0.60m)
   2               : Climb +300mm step onto Level 2 (+0.90m)
   R               : Move to Red L1 Retry Zone (-1.85, 2.45, z=0.60m)
   M               : Toggle MANUAL Mode <-> AUTONOMOUS A* Follower Mode
   Ctrl+C          : Quit
========================================================================
"""


def get_key(settings) -> str:
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            seq = sys.stdin.read(2)
            ch += seq
    else:
        ch = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return ch


def main(args=None) -> None:
    if not sys.stdin.isatty():
        print('teleop_keyboard requires an interactive terminal.')
        return

    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = Node('br_teleop_keyboard')
    cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    climb_pub = node.create_publisher(String, '/br_climb_command', 10)

    def send_climb(text: str) -> None:
        m = String()
        m.data = text
        climb_pub.publish(m)

    def send_twist(vx: float, vy: float, wz: float) -> None:
        t = Twist()
        t.linear.x = vx
        t.linear.y = vy
        t.angular.z = wz
        cmd_pub.publish(t)

    print(BANNER)
    speed = 0.70
    turn = 1.20
    auto_mode = False

    try:
        while rclpy.ok():
            key = get_key(settings)
            if not key:
                continue
            if key == '\x03':  # Ctrl+C
                break

            low = key.lower()
            if low == 'w' or key == '\x1b[A':
                send_climb('mode_manual')
                send_twist(speed, 0.0, 0.0)
            elif low == 's' or key == '\x1b[B':
                send_climb('mode_manual')
                send_twist(-speed, 0.0, 0.0)
            elif low == 'a' or key == '\x1b[D':
                send_climb('mode_manual')
                send_twist(0.0, speed, 0.0)
            elif low == 'd' or key == '\x1b[C':
                send_climb('mode_manual')
                send_twist(0.0, -speed, 0.0)
            elif low == 'q':
                send_climb('mode_manual')
                send_twist(0.0, 0.0, turn)
            elif low == 'e':
                send_climb('mode_manual')
                send_twist(0.0, 0.0, -turn)
            elif low == ' ':
                send_twist(0.0, 0.0, 0.0)
            elif low == 'l':
                send_climb('climb_up')
            elif low == 'k':
                send_climb('climb_down')
            elif low == '0':
                send_climb('ground')
            elif low == '1':
                send_climb('level_1')
            elif low == '2':
                send_climb('level_2')
            elif low == 'r':
                send_climb('l1_retry')
            elif low == 'm':
                auto_mode = not auto_mode
                send_climb('mode_auto' if auto_mode else 'mode_manual')
    finally:
        send_twist(0.0, 0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
