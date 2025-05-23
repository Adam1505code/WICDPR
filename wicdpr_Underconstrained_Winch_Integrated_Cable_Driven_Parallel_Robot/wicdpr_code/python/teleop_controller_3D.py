#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray
import sys, select, termios, tty
import time

DURATION = 1.0  # seconds per command

settings = termios.tcgetattr(sys.stdin)

# Key bindings for cable robot (FR, FL, RL, RR order)
move_bindings = {
    'w': [ 20,  20, -20, -20],  # North  (Y+): Pull front cables in, rear out
    's': [-20, -20,  20,  20],  # South  (Y-): Pull rear in, front out
    'a': [ 20, -20,  20, -20],  # West   (X-): Pull left in, right out
    'd': [-20,  20, -20,  20],  # East   (X+): Pull right in, left out
    'q': [ 20,  20,  20,  20],  # Up     (Z+): All cables in
    'e': [-20, -20, -20, -20],  # Down   (Z-): All cables out
    ' ': [0, 0, 0, 0],          # Stop all
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
    rospy.init_node('teleop_cable3d')
    pub = rospy.Publisher('/motor_speeds', Int32MultiArray, queue_size=1)

    print("""
    Cable Robot 3D Teleop (1 second per command)
    --------------------------------------------
    w: North      s: South
    a: West       d: East
    q: Up         e: Down
    SPACE: Stop

    CTRL+C to quit
    """)

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key in move_bindings:
                speeds = move_bindings[key]
                msg = Int32MultiArray()
                msg.data = speeds
                pub.publish(msg)
                print(f"Sent: {speeds} for {DURATION} second(s)")
                t0 = time.time()
                while time.time() - t0 < DURATION and not rospy.is_shutdown():
                    rospy.sleep(0.05)
                msg_stop = Int32MultiArray()
                msg_stop.data = [0, 0, 0, 0]
                pub.publish(msg_stop)
                print("Motors stopped.")
            elif key == '\x03':
                break
    except Exception as e:
        print(e)
    finally:
        msg = Int32MultiArray()
        msg.data = [0, 0, 0, 0]
        pub.publish(msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
