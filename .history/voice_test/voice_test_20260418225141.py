#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8
import sys
import select
import termios
import tty

class VoiceSwitchPublisher:
    def __init__(self):
        rospy.init_node('voice_switch_keyboard', anonymous=True)
        self.pub = rospy.Publisher('/voice_switch', UInt8, queue_size=10)
        self.voice_switch = 40
        self.rate = rospy.Rate(10)  # 10 Hz

        # 设置终端为非阻塞模式，读取键盘
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        print("Press 'w' to increase, 'q' to decrease, Ctrl+C to exit.")

    def getKey(self):
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            return sys.stdin.read(1)
        return None

    def run(self):
        try:
            while not rospy.is_shutdown():
                key = self.getKey()
                if key == 'w':
                    self.voice_switch += 1
                    print(f"Increased to {self.voice_switch}")
                elif key == 'q':
                    self.voice_switch -= 1
                    if self.voice_switch < 0:
                        self.voice_switch = 0
                    print(f"Decreased to {self.voice_switch}")

                msg = UInt8()
                msg.data = self.voice_switch
                self.pub.publish(msg)

                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

if __name__ == '__main__':
    node = VoiceSwitchPublisher()
    node.run()
