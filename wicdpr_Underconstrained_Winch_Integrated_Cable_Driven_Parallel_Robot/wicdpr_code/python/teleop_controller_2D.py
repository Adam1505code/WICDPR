#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import time

DURATION = 1.0  # seconds per command

settings = termios.tcgetattr(sys.stdin)

# Key bindings: left = linear.y, right = linear.x
move_bindings = {
    'w': (-0.2,  0.2),  # Up: both cables in (raise platform)
    's': ( 0.2, -0.2),  # Down: both cables out (lower platform)
    'a': (-0.2, 0.0),   # Left: left cable in only
    'd': (0.0, 0.2),    # Right: right cable in only
    ' ': (0.0,  0.0),   # Stop
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    rospy.init_node('teleop_cable2d')
    pub = rospy.Publisher('/cmd_cable_lengths', Twist, queue_size=1)

    print("""
    Cable Robot 2D Teleop (1 second per command)
    --------------------------------------------
    w: Up      s: Down
    a: Left    d: Right
    SPACE: Stop

    CTRL+C to quit
    """)

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key in move_bindings:
                left, right = move_bindings[key]
                msg = Twist()
                msg.linear.y = left
                msg.linear.x = right
                pub.publish(msg)
                print(f"Sent: left={left}, right={right} for {DURATION} second(s)")
                t0 = time.time()
                while time.time() - t0 < DURATION and not rospy.is_shutdown():
                    rospy.sleep(0.05)
                msg_stop = Twist()
                msg_stop.linear.y = 0.0
                msg_stop.linear.x = 0.0
                pub.publish(msg_stop)
                print("Motors stopped.")
            elif key == '\x03':
                break
    except Exception as e:
        print(e)
    finally:
        msg = Twist()
        msg.linear.y = 0.0
        msg.linear.x = 0.0
        pub.publish(msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
