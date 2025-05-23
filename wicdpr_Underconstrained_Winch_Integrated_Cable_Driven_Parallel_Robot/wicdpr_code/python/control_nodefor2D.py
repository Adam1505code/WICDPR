#!/usr/bin/env python3
"""
2-D cable-platform controller
 • Ziegler–Nichols PID (example values)
 • 12° roll offset
 • Per-motor direction flags (invert_left, invert_right)
 • Publishes geometry_msgs/Twist on /cmd_cable_lengths:
     - left motor => twist.linear.y
     - right motor => twist.linear.x
 • Logs all relevant data to a CSV file for debugging.
 • Now includes an enhanced E-stop logic.
"""

import csv
import numpy as np
import rospy
from datetime import datetime
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

class CablePlatform2DController:
    def __init__(self, W=1.3, H=0.76, sx=0.65, sz=0.38, ex=None, ez=None):
        """Initialize the 2D platform controller."""
        # ── Configuration dictionary ─────────────────────────────
        self.config = {
            'workspace': {
                'width':  W,
                'height': H
            },
            'platform': {
                'width':    0.30,   # Not used in this simplified 2D code
                'mass':     2.7,    # kg
                'start_pos': [sx, sz]
            },
            'cables': {
                'pulley_diameter': 0.01054,
                'ticks_per_rev':   1600,
                # user wants max_speed ~ 0.09 m/s
                'max_speed':       0.09,
                'invert_left':     True,   # Adjust as needed
                'invert_right':    False
            },
            'safety': {
                'e_stop_topic': '/emergency_stop',
                'max_tension':  100.0  # not fully used in this example
            },
            'control': {
                'base_speed': 0.10  # m/s (nominal retraction speed)
            },
            'logging': {
                'enabled':  True,
                'filename': f'platform_log_{datetime.now():%Y%m%d_%H%M%S}.csv'
            }
        }

        # Optional "end position" if you want to do path/motion planning
        self.end_pos = [ex, ez] if (ex is not None and ez is not None) else None

        # ── Constants & State ─────────────────────────────────────
        self.g = 9.81
        self.m_per_tick = (np.pi * self.config['cables']['pulley_diameter']
                           / self.config['cables']['ticks_per_rev'])
        self.roll_offset_deg = 12.0  # subtract from measured roll

        self.e_stop = False

        # Cable lengths in meters (from your encoder topics)
        self.cable_lengths = [0.0, 0.0]  # [L1, L2]

        # Platform position (x,z) in meters
        self.platform_pos = [sx, sz]

        # IMU angles (in degrees, from the callbacks, and in radians for internal usage)
        self.pitch_deg_raw = 0.0
        self.roll_deg_raw  = 0.0   # raw before offset
        self.pitch_rad     = 0.0   # final used for control
        self.roll_rad      = 0.0   # final used for control

        # For debugging/logging: store the last motor commands
        self.last_cmd_left  = 0.0
        self.last_cmd_right = 0.0

        # ── Ziegler–Nichols PID tuning (Example Values) ──────────
        # Suppose we measured:
        #   Ku = 2.0   (ultimate gain)
        #   Pu = 1.0   (ultimate period)
        # Then the classic Z-N formulas for a PID are:
        #   Kp = 0.6 Ku = 1.2
        #   Ki = 1.2 Ku / Pu = 2.4
        #   Kd = 0.075 Ku * Pu = 0.15
        Ku = 2.0
        Pu = 1.0
        self.pid = {
            'kp': 0.6 * Ku,                      # e.g. 1.2
            'ki': 2.0 * (0.6 * Ku) / Pu,         # e.g. 2.4
            'kd': 0.075 * Ku * Pu                # e.g. 0.15
        }

        # Integrator & state for the PID
        self.i_err = 0.0
        self.last_err = 0.0

        # ── ROS initialization ────────────────────────────────────
        rospy.init_node('cable_platform_2d_controller')
        self._ros_io()
        self._setup_log()

        rospy.loginfo(f"Workspace: W={W}m, H={H}m | Start=(x={sx}, z={sz}), End={self.end_pos}")
        rospy.loginfo(f"Z-N PID Gains => {self.pid} | roll offset = -{self.roll_offset_deg}°")
        rospy.loginfo("Invert left=%s, right=%s" % (self.config['cables']['invert_left'],
                                                    self.config['cables']['invert_right']))
        rospy.loginfo(f"Max motor speed set to {self.config['cables']['max_speed']:.3f} m/s")

    # ─────────────────────────────────────────────────────────────
    #   ROS wiring: subscribers & publisher
    # ─────────────────────────────────────────────────────────────
    def _ros_io(self):
        # IMU
        rospy.Subscriber('/imu/pitch', Float32, self._pitch_cb)
        rospy.Subscriber('/imu/roll',  Float32, self._roll_cb)

        # Cable lengths
        rospy.Subscriber('/cable_length/0', Float32, lambda m: self._set_len(0, m.data))
        rospy.Subscriber('/cable_length/1', Float32, lambda m: self._set_len(1, m.data))

        # Emergency Stop
        rospy.Subscriber(self.config['safety']['e_stop_topic'], Bool, self._e_stop_cb)

        # Command publisher: geometry_msgs/Twist
        #   - left motor => twist.linear.y
        #   - right motor => twist.linear.x
        self.cmd_pub = rospy.Publisher('/cmd_cable_lengths', Twist, queue_size=10)

    def _pitch_cb(self, msg):
        """
        We get pitch in degrees from Arduino. Store raw degrees, convert to radians.
        """
        self.pitch_deg_raw = msg.data
        self.pitch_rad = np.deg2rad(self.pitch_deg_raw)

    def _roll_cb(self, msg):
        """
        We get roll in degrees from Arduino. We do a 12° offset, then store raw & final rad.
        """
        self.roll_deg_raw = msg.data
        # Apply the offset: final roll = raw - 12 deg
        offset_roll_deg = self.roll_deg_raw - self.roll_offset_deg
        self.roll_rad = np.deg2rad(offset_roll_deg)

    def _set_len(self, i, val):
        """
        i=0 => left cable length, i=1 => right cable length
        (or vice versa, but be consistent with your hardware).
        """
        self.cable_lengths[i] = val

    def _e_stop_cb(self, msg):
        prev_e_stop = self.e_stop
        self.e_stop = msg.data

        if self.e_stop and not prev_e_stop:
            rospy.logwarn("EMERGENCY STOP activated: motor speeds forced to 0.")
        elif (not self.e_stop) and prev_e_stop:
            rospy.loginfo("E-stop released: normal operation resumed.")

    # ─────────────────────────────────────────────────────────────
    #   CSV Logging
    # ─────────────────────────────────────────────────────────────
    def _setup_log(self):
        """
        Create a CSV file if logging is enabled. We'll log every relevant piece of info
        in each iteration of the control loop.
        """
        if not self.config['logging']['enabled']:
            return
        self.log_fh = open(self.config['logging']['filename'], 'w', newline='')
        self.csv_writer = csv.writer(self.log_fh)
        # CSV header:
        self.csv_writer.writerow([
            "time_sec",
            "platform_x_m", "platform_z_m",        # Calculated platform position
            "pitch_deg_raw", "roll_deg_raw",       # from Arduino directly
            "pitch_rad_final", "roll_rad_final",   # after offsets, used in control
            "cableLen_left", "cableLen_right",     # cable lengths
            "cmdLeft_m_s", "cmdRight_m_s",         # final motor speeds
            "Kp", "Ki", "Kd",
            "e_stop"                               # 1 if active, else 0
        ])

    def _log_csv(self):
        """Write one row of data per loop iteration."""
        if not self.config['logging']['enabled']:
            return

        t = rospy.get_time()  # current ROS time in seconds
        x, z = self.platform_pos
        L0, L1 = self.cable_lengths

        row = [
            t,
            x, z,
            self.pitch_deg_raw,  # raw pitch from Arduino
            self.roll_deg_raw,   # raw roll from Arduino
            self.pitch_rad,      # final pitch used in control
            self.roll_rad,       # final roll used in control
            L0,                  # left cable
            L1,                  # right cable
            self.last_cmd_left,  # final commanded speed to left motor
            self.last_cmd_right, # final commanded speed to right motor
            self.pid["kp"], self.pid["ki"], self.pid["kd"],
            int(self.e_stop)
        ]
        self.csv_writer.writerow(row)
        self.log_fh.flush()

    # ─────────────────────────────────────────────────────────────
    #   Forward kinematics, PID, and command sender
    # ─────────────────────────────────────────────────────────────
    def _calc_pos(self):
        """
        Very simple 2-anchor geometry:
          • Anchors at (0, H) and (W, H).
          • cable_lengths[0] = distance to left anchor
          • cable_lengths[1] = distance to right anchor
        Solve for platform (x,z).
        """
        L1, L2 = self.cable_lengths
        W = self.config['workspace']['width']
        H = self.config['workspace']['height']

        # from typical cable geometry: x = (L2^2 - L1^2 - W^2)/(-2W)
        x = (L2**2 - L1**2 - W**2) / (-2.0 * W)

        inside = L1**2 - x**2
        if inside < 0:
            inside = 0.0
        z = np.sqrt(inside) + H

        self.platform_pos = [x, z]

    def _tilt_pid(self):
        """Simple PID on pitch angle (in radians). Negative sign to oppose tilt."""
        # We want to keep pitch=0 => error = -pitch
        err = -self.pitch_rad

        self.i_err += err
        d_err = err - self.last_err
        self.last_err = err

        p = self.pid
        return (p['kp'] * err) + (p['ki'] * self.i_err) + (p['kd'] * d_err)

    def _send(self, left_speed, right_speed):
        """
        Publish geometry_msgs/Twist to /cmd_cable_lengths.
         - left  => twist.linear.y
         - right => twist.linear.x
        Apply inversion, clipping, and E-stop if needed.
        """
        if self.e_stop:
            left_speed  = 0.0
            right_speed = 0.0

        # Invert if needed
        if self.config['cables']['invert_left']:
            left_speed = -left_speed
        if self.config['cables']['invert_right']:
            right_speed = -right_speed

        # Clip to max speed
        vmax = self.config['cables']['max_speed']
        left_speed  = float(np.clip(left_speed,  -vmax, vmax))
        right_speed = float(np.clip(right_speed, -vmax, vmax))

        # Build Twist
        tw = Twist()
        # Arduino expects:
        #   left  = msg.linear.y
        #   right = msg.linear.x
        tw.linear.y = left_speed
        tw.linear.x = right_speed

        self.cmd_pub.publish(tw)

        # Store for logging
        self.last_cmd_left  = left_speed
        self.last_cmd_right = right_speed

    # ─────────────────────────────────────────────────────────────
    #   Main loop
    # ─────────────────────────────────────────────────────────────
    def run(self):
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            # 1. Forward kinematics to find (x,z)
            self._calc_pos()

            # 2. Compute tilt correction from pitch
            corr = self._tilt_pid()

            # 3. Combine with a base speed for both motors
            base = self.config['control']['base_speed']
            left_cmd  = base + corr
            right_cmd = base - corr

            # 4. Send command (auto-check e-stop inside _send())
            self._send(left_cmd, right_cmd)

            # 5. Log the data to CSV
            self._log_csv()

            rate.sleep()


# ─────────────────────────────────────────────────────────────
#   Script entry point
# ─────────────────────────────────────────────────────────────
if __name__ == '__main__':
    try:
        # Simple interactive prompts
        W  = float(input("Workspace width (m) [1.3]: ") or 1.3)
        H  = float(input("Workspace height (m) [0.76]: ") or 0.76)
        sx = float(input("Start X (m) [0.65]: ") or 0.65)
        sz = float(input("Start Z (m) [0.38]: ") or 0.38)
        ex = input("End X (m) [skip]: ")
        ez = input("End Z (m) [skip]: ")
        ex = float(ex) if ex else None
        ez = float(ez) if ez else None

        controller = CablePlatform2DController(W, H, sx, sz, ex, ez)
        controller.run()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Exception in controller: {e}")
