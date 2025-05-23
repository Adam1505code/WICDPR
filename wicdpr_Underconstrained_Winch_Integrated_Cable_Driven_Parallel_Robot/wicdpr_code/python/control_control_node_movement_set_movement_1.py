#!/usr/bin/env python3
import rospy
import numpy as np
import csv
from datetime import datetime
from std_msgs.msg import Float32, Int32MultiArray, Bool
from geometry_msgs.msg import Vector3
# from pidautotune import PIDAutotune  # Not strictly needed if we're just assigning Z-N gains directly

class CablePlatform3DController:
    def __init__(self):
        # Configuration parameters
        self.config = {
            'workspace': {
                'width': 1.3,    # x-dimension (meters)
                'depth': 1.3,     # y-dimension (meters)
                'height': 0.76    # z-dimension (meters)
            },
            'platform': {
                'width': 0.3,     # x-dimension (meters)
                'depth': 0.3,     # y-dimension (meters)
                'mass': 2.7,      # kg
                'start_pos': [0.65, 0.65, 0.38]  # [x, y, z]
            },
            'motors': {
                'layout': {
                    1: 'front-right',
                    2: 'front-left',
                    3: 'rear-left',
                    4: 'rear-right'
                },
                'max_speed': 50
            },
            'safety': {
                'e_stop_topic': '/emergency_stop',
                'max_tension': 100  # Newtons (safety threshold)
            },
            'logging': {
                'enabled': True,
                'filename': f'platform_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
            }
        }

        # System state
        self.g = 9.81
        self.e_stop = False
        self.cable_lengths = [0, 0, 0, 0]  # [fr, fl, rl, rr]
        self.orientation = {'pitch': 0, 'roll': 0}
        self.platform_pos = self.config['platform']['start_pos']

        # -------- ZIEGLER–NICHOLS PID GAINS (shared by pitch & roll) --------
        # Suppose Ku=2.0, Pu=1.0:
        #   Kp = 0.6*Ku=1.2
        #   Ki = 2*(0.6*Ku)/Pu=2.4
        #   Kd = 0.075*Ku*Pu=0.15
        Ku = 2.0
        Pu = 1.0
        zn_kp = 0.6 * Ku  # 1.2
        zn_ki = 2.0 * zn_kp / Pu  # 2.4
        zn_kd = 0.075 * Ku * Pu   # 0.15

        # Assign them to pitch and roll
        self.pid_gains = {
            'pitch': {'kp': zn_kp, 'ki': zn_ki, 'kd': zn_kd},
            'roll':  {'kp': zn_kp, 'ki': zn_ki, 'kd': zn_kd}
        }

        # PID integrator states
        self.integral_errors = {'pitch': 0, 'roll': 0}
        self.last_errors = {'pitch': 0, 'roll': 0}

        # ROS initialization
        rospy.init_node('platform_3d_controller')
        self.setup_ros_communications()

        # Data logging
        if self.config['logging']['enabled']:
            self.setup_data_logging()

        rospy.loginfo("3D Controller initialized with configuration:")
        rospy.loginfo(f"Workspace: {self.config['workspace']}")
        rospy.loginfo(f"Platform: {self.config['platform']}")
        rospy.loginfo(f"Motor layout: {self.config['motors']['layout']}")

        rospy.loginfo("Using Ziegler–Nichols PID Gains => "
                      f"Kp={zn_kp:.2f}, Ki={zn_ki:.2f}, Kd={zn_kd:.2f} "
                      "(pitch & roll)")

    def setup_ros_communications(self):
        # Subscribers
        rospy.Subscriber('/imu/pitch', Float32, self.pitch_callback)
        rospy.Subscriber('/imu/roll', Float32, self.roll_callback)
        rospy.Subscriber('/encoder/0', Float32, self.fr_encoder_callback)  # Front-right
        rospy.Subscriber('/encoder/1', Float32, self.fl_encoder_callback)  # Front-left
        rospy.Subscriber('/encoder/2', Float32, self.rl_encoder_callback)  # Rear-left
        rospy.Subscriber('/encoder/3', Float32, self.rr_encoder_callback)  # Rear-right
        rospy.Subscriber(self.config['safety']['e_stop_topic'], Bool, self.e_stop_callback)

        # Publishers
        self.motor_pub = rospy.Publisher('/motor_speeds', Int32MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher('/controller_status', Vector3, queue_size=10)
        self.tension_pub = rospy.Publisher('/cable_tensions', Float32, queue_size=10)

    def setup_data_logging(self):
        self.log_file = open(self.config['logging']['filename'], 'w')
        self.log_writer = csv.writer(self.log_file)
        headers = [
            'timestamp', 'pos_x', 'pos_y', 'pos_z',
            'pitch', 'roll', 'tension_fr', 'tension_fl',
            'tension_rl', 'tension_rr', 'motor_fr', 'motor_fl',
            'motor_rl', 'motor_rr', 'pid_pitch_kp', 'pid_pitch_ki',
            'pid_pitch_kd', 'pid_roll_kp', 'pid_roll_ki', 'pid_roll_kd'
        ]
        self.log_writer.writerow(headers)
        rospy.loginfo(f"Data logging to {self.config['logging']['filename']}")

    def log_data(self):
        if not self.config['logging']['enabled']:
            return

        data = [
            rospy.get_time(),
            self.platform_pos[0],
            self.platform_pos[1],
            self.platform_pos[2],
            np.rad2deg(self.orientation['pitch']),
            np.rad2deg(self.orientation['roll']),
            *getattr(self, 'current_tensions', [0,0,0,0]),  # if not defined yet, default
            *getattr(self, 'last_motor_cmd', [0,0,0,0]),
            self.pid_gains['pitch']['kp'],
            self.pid_gains['pitch']['ki'],
            self.pid_gains['pitch']['kd'],
            self.pid_gains['roll']['kp'],
            self.pid_gains['roll']['ki'],
            self.pid_gains['roll']['kd']
        ]
        self.log_writer.writerow(data)
        self.log_file.flush()

    def pitch_callback(self, msg):
        self.orientation['pitch'] = np.deg2rad(msg.data)
        self.report_status("IMU", f"Pitch: {msg.data:.2f}°")

    def roll_callback(self, msg):
        self.orientation['roll'] = np.deg2rad(msg.data)
        self.report_status("IMU", f"Roll: {msg.data:.2f}°")

    def fr_encoder_callback(self, msg):
        self.cable_lengths[0] = msg.data * 0.001  # Front-right
        self.report_status("Encoder FR", f"Length: {self.cable_lengths[0]:.3f}m")

    def fl_encoder_callback(self, msg):
        self.cable_lengths[1] = msg.data * 0.001  # Front-left
        self.report_status("Encoder FL", f"Length: {self.cable_lengths[1]:.3f}m")

    def rl_encoder_callback(self, msg):
        self.cable_lengths[2] = msg.data * 0.001  # Rear-left
        self.report_status("Encoder RL", f"Length: {self.cable_lengths[2]:.3f}m")

    def rr_encoder_callback(self, msg):
        self.cable_lengths[3] = msg.data * 0.001  # Rear-right
        self.report_status("Encoder RR", f"Length: {self.cable_lengths[3]:.3f}m")

    def e_stop_callback(self, msg):
        self.e_stop = msg.data
        if self.e_stop:
            self.send_motor_command([0, 0, 0, 0])
            rospy.logerr("!!! EMERGENCY STOP ACTIVATED !!!")
        else:
            rospy.logwarn("Emergency stop released")

    def report_status(self, source, message):
        status_msg = Vector3()
        status_msg.x = rospy.get_time()
        status_msg.y = hash(source) % 1000
        status_msg.z = hash(message) % 1000
        self.status_pub.publish(status_msg)
        rospy.loginfo(f"[{source}] {message}")

    def calculate_position(self):
        """3D forward kinematics using cable lengths (simplified)."""
        anchors = np.array([
            [self.config['workspace']['width'], 0, self.config['workspace']['height']],  # FR
            [0, 0, self.config['workspace']['height']],                                  # FL
            [0, self.config['workspace']['depth'], self.config['workspace']['height']],  # RL
            [self.config['workspace']['width'], self.config['workspace']['depth'], self.config['workspace']['height']]  # RR
        ])
        # Very simplified approach: average anchor coords minus cable lengths:
        x = np.mean([a[0] for a in anchors])
        y = np.mean([a[1] for a in anchors])
        z = np.mean([a[2] - l for a, l in zip(anchors, self.cable_lengths)])
        self.platform_pos = [x, y, z]

    def calculate_tensions(self):
        """Compute cable tensions using 3D static equilibrium (simplified)."""
        anchors = np.array([
            [self.config['workspace']['width'], 0, self.config['workspace']['height']],  # FR
            [0, 0, self.config['workspace']['height']],                                  # FL
            [0, self.config['workspace']['depth'], self.config['workspace']['height']],  # RL
            [self.config['workspace']['width'], self.config['workspace']['depth'], self.config['workspace']['height']]  # RR
        ])
        platform_corners = np.array([
            [self.platform_pos[0] + 0.15, self.platform_pos[1] - 0.15, self.platform_pos[2]],
            [self.platform_pos[0] - 0.15, self.platform_pos[1] - 0.15, self.platform_pos[2]],
            [self.platform_pos[0] - 0.15, self.platform_pos[1] + 0.15, self.platform_pos[2]],
            [self.platform_pos[0] + 0.15, self.platform_pos[1] + 0.15, self.platform_pos[2]]
        ])
        cable_vectors = anchors - platform_corners
        norms = np.linalg.norm(cable_vectors, axis=1)
        unit_vectors = cable_vectors / norms[:, np.newaxis]

        A = np.vstack([unit_vectors.T,
                       np.cross(platform_corners - self.platform_pos, unit_vectors).T])
        b = np.array([0, 0, self.config['platform']['mass'] * self.g, 0, 0, 0])

        tensions = np.linalg.pinv(A) @ b
        tensions = np.clip(tensions, 0, self.config['safety']['max_tension'])
        self.current_tensions = tensions
        return tensions

    def tilt_compensation(self):
        """Apply the Z-N PID on pitch & roll."""
        corrections = {}
        for axis in ['pitch', 'roll']:
            error = -self.orientation[axis]  # Negative to oppose tilt
            # PID terms
            P = self.pid_gains[axis]['kp'] * error
            self.integral_errors[axis] += error
            I = self.pid_gains[axis]['ki'] * self.integral_errors[axis]
            D = self.pid_gains[axis]['kd'] * (error - self.last_errors[axis])
            self.last_errors[axis] = error
            corrections[axis] = P + I + D

        # Motor corrections [FR, FL, RL, RR]
        motor_corrections = [
            corrections['pitch'] - corrections['roll'],   # front-right
            corrections['pitch'] + corrections['roll'],   # front-left
            -corrections['pitch'] + corrections['roll'],  # rear-left
            -corrections['pitch'] - corrections['roll']   # rear-right
        ]
        return motor_corrections

    def send_motor_command(self, speeds):
        """Send speed commands (Int32MultiArray) to /motor_speeds, with E-stop & clipping."""
        if self.e_stop:
            speeds = [0, 0, 0, 0]

        speeds = [int(np.clip(s, -self.config['motors']['max_speed'],
                              self.config['motors']['max_speed'])) for s in speeds]

        cmd = Int32MultiArray()
        cmd.data = speeds
        self.motor_pub.publish(cmd)
        self.last_motor_cmd = speeds

        self.report_status("Motor Cmd",
                           f"FR={speeds[0]}, FL={speeds[1]}, RL={speeds[2]}, RR={speeds[3]}")

    def run(self):
        rate = rospy.Rate(50)  # 50Hz control loop
        try:
            while not rospy.is_shutdown():
                if not self.e_stop:
                    # 1) Position (very simple approximate)
                    self.calculate_position()
                    # 2) Tensions
                    tensions = self.calculate_tensions()
                    if any(t > self.config['safety']['max_tension'] for t in tensions):
                        rospy.logwarn(f"High tension: {tensions}")
                        self.send_motor_command([0, 0, 0, 0])
                        rate.sleep()
                        continue
                    # 3) tilt compensation
                    corr = self.tilt_compensation()
                    base_speed = 0  # can be changed for actual movement
                    motor_speeds = [base_speed + c for c in corr]
                    # 4) send
                    self.send_motor_command(motor_speeds)
                # 5) log
                self.log_data()
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
        finally:
            if self.config['logging']['enabled']:
                self.log_file.close()
            self.send_motor_command([0,0,0,0])
            rospy.loginfo("Controller shutdown complete")

if __name__ == '__main__':
    try:
        controller = CablePlatform3DController()
        controller.run()
    except Exception as e:
        rospy.logerr(f"Controller error: {str(e)}")
