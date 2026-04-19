#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
==== 四轮舵机小车 最终完美版 ====
W/S : 前进/后退（长按流畅）
A/D : 左转/右转（倒车正向）
空格 : 立即停止
Q    : 退出
==================================
"""

FORWARD_SPEED = 0.4
BACKWARD_SPEED = -0.3
STEER_VALUE = 0.6

def getKey(timeout):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, w, e = select.select([fd], [], [], timeout)
        key = sys.stdin.read(1) if r else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return key

def main():
    rospy.init_node('wsad_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    twist = Twist()
    print(msg)

    linear = 0.0
    angular = 0.0

    try:
        while not rospy.is_shutdown():
            key = getKey(0.05)

            # ================== 速度（只更新，不自动清零）
            if key == 'w':
                linear = FORWARD_SPEED
            elif key == 's':
                linear = BACKWARD_SPEED
            elif key == ' ':
                linear = 0
                angular = 0
            elif key == 'q':
                break

            # ================== 转向（只更新，不影响速度）
            if key == 'a':
                angular = STEER_VALUE
            elif key == 'd':
                angular = -STEER_VALUE

            # 倒车时自动反转转向
            if linear < 0:
                twist.angular.z = -angular
            else:
                twist.angular.z = angular
            
            twist.linear.x = linear
            pub.publish(twist)

    except Exception as e:
        print(e)

    twist = Twist()
    pub.publish(twist)
    print("\n✅ 已停止")

if __name__ == '__main__':
    main()