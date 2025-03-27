#!/usr/bin/env python3

import rospy
import numpy as np
import math
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from traj_analyzer.msg import RefTraj
from std_msgs.msg import Float64MultiArray, Float64, Bool, Int32
from dynamic_reconfigure.server import Server
from car_controller_ros.cfg import LongitudePIDParamsConfig
from collections import deque
import os

class LateralController:
    def __init__(self):
        rospy.init_node('lateral_controller', anonymous=False)
        
        # Get parameters
        self.wheelbase = rospy.get_param('~wheelbase', 0.174)  # Vehicle wheelbase
        self.control_rate = rospy.get_param('~control_rate', 20.0)  # Control frequency
        self.max_steering_angle = rospy.get_param('~max_steering_angle', 25.0)  # Maximum steering angle (degrees)
        self.max_steering_rad = math.radians(self.max_steering_angle)
        
        # Trajectory management parameters
        self.fifo_size = rospy.get_param('~fifo_size', 400)
        self.traj_file_path = rospy.get_param('~traj_file_path', 'traj_analyzer/refine_traj_fast.txt')
        self.frame_id = rospy.get_param('~frame_id', 'odom_capture_frame')
        
        # Trajectory management variables
        self.all_trajectory_points = []  # Store all trajectory points
        self.fifo_buffer = deque(maxlen=self.fifo_size)  # FIFO buffer
        self.current_traj_idx = 0
        
        # State variables
        self.current_pose = None
        self.current_velocity = 0.0
        self.current_yaw_rate = 0.0
        self.y_dot = 0.0
        self.prev_steering = 0.0
        
        # E-stop state
        self.estop_active = False
        self.safety_violation = False
        
        # Safety limits
        self.max_lateral_error = rospy.get_param('~max_lateral_error', 0.5)
        self.max_heading_error = rospy.get_param('~max_heading_error', 30.0)
        
        # Control gain matrix - obtained from MATLAB code
        self.K_gains = np.array([
            [-5.05343693359301,-0.0959301262792274,-1.60383689965672,-0.0163177101700867],
            [-4.33835189598493,-0.0899444054449210,-1.49113131030043,-0.0170777303353192],
            [-3.78573260687361,-0.0850323367442463,-1.39878024709001,-0.0177833448928725],
            [-3.34814069813036,-0.0809338811665958,-1.32188697113860,-0.0184474848492566],
            [-2.99488568230008,-0.0774784136985286,-1.25703759949569,-0.0190790203810848],
            [-2.81166282794989,-0.0778036155443890,-1.22751394824941,-0.0199713204083865],
            [-2.55808138804857,-0.0751072216307103,-1.17854127823498,-0.0205631778235457],
            [-2.25828339580164,-0.0697956807025532,-1.11294699405489,-0.0208358676835320],
            [-2.00773448764947,-0.0650936942493804,-1.05498129577411,-0.0210781499716144],
            [-1.86410212619741,-0.0635259105853891,-1.02437640543957,-0.0216134075189041],
            [-1.67697392320236,-0.0596233948405479,-0.977283705442289,-0.0218161383022824],
            [-1.51625064747038,-0.0561068345469056,-0.934882307838881,-0.0219997631796445],
            [-1.42793354354891,-0.0551898188287044,-0.915101985032981,-0.0225027008039230],
            [-1.30224273394362,-0.0521833571627288,-0.879561587274598,-0.0226614035311454],
            [-1.19210686834890,-0.0494398150000221,-0.847128601606196,-0.0228072641002867],
            [-1.09503568735318,-0.0469258878112629,-0.817417574712336,-0.0229419496112648],
            [-1.04517383275862,-0.0465243058959571,-0.806408343189399,-0.0234198282002949],
            [-0.965799750384190,-0.0443122997607369,-0.780757010539154,-0.0235401274552425],
            [-0.894828420557652,-0.0422671485759989,-0.757036590660418,-0.0236522268255630],
            [-0.831108148847569,-0.0403704910984141,-0.735044623114165,-0.0237573689759519],
            [-0.773675493582585,-0.0386080262332857,-0.714603506663381,-0.0238559167588535],
            [-0.746789464665513,-0.0385540943524211,-0.709890500623783,-0.0243179102589142],
            [-0.697913597889053,-0.0369616823211541,-0.691782750053527,-0.0244085378213624],
            [-0.653427520496908,-0.0354724490745596,-0.674850549234679,-0.0244946002855827],
            [-0.592352130419241,-0.0326484198130086,-0.645557196270125,-0.0241986052637124],
            [-0.614765935102071,-0.0356045750606346,-0.670550742436906,-0.0254072840974208],
            [-0.559569979827022,-0.0328858887378749,-0.643005997446479,-0.0251077219393352],
            [-0.544382266648718,-0.0330213362734576,-0.642345539040967,-0.0255611566735593],
            [-0.513217159040605,-0.0318349914339547,-0.629432435514301,-0.0256330999004156],
            [-0.438748578764292,-0.0269894475585669,-0.580327359868689,-0.0245410500800681],
            [-0.472407519931497,-0.0308983556280456,-0.617899850430289,-0.0261534562684632],
            [-0.446690082650718,-0.0298493216019248,-0.606762370189187,-0.0262187455672089],
            [-0.422726498433036,-0.0288494719211414,-0.596192823437878,-0.0262819295627051],
            [-0.423079201760851,-0.0299385573098872,-0.599623841290695,-0.0266848609683923],
            [-0.375516847819688,-0.0265559077647763,-0.565831928219194,-0.0259405809666788],
            [-0.354441618581577,-0.0254923943112203,-0.551481486161367,-0.0257609140149112],
            [-0.334260389130784,-0.0244218250317978,-0.537248764630869,-0.0255653871518884],
            [-0.335864666690809,-0.0254221880093017,-0.541204032577295,-0.0259463304929839],
            [-0.316441684383003,-0.0242942461621734,-0.526963145727946,-0.0257240954166837],
            [-0.297909609567305,-0.0231731460849310,-0.512874117927077,-0.0254879865154979]
        ])
        
        self.L_gains = np.array([
            -0.116278136329070,
            -0.0980505191793439,
            -0.0841406581806088,
            -0.0732719131904196,
            -0.0645908719677455,
            -0.0430002618980076,
            -0.0373677730397041,
            -0.0319707327909971,
            -0.0315911853960587,
            -0.0277072202702114,
            -0.0289435436762753,
            -0.0255718934244124,
            -0.0242694952863533,
            -0.0232387432962793,
            -0.0215737172045934,
            -0.0200610123589473,
            -0.0189835946164580,
            -0.0178130604562318,
            -0.0167568970816786,
            -0.0159926036030597,
            -0.0150140156429866,
            -0.0145436660344001,
            -0.0137736517796197,
            -0.0130890636168592,
            -0.0122186124333131,
            -0.0124402992253541,
            -0.0115487559166845,
            -0.0113670349815845,
            -0.0108511815603949,
            -0.00924408865595257,
            -0.0102162236425890,
            -0.00979015633853910,
            -0.00939238589245627,
            -0.00928750407540350,
            -0.00846794522942772,
            -0.00784672464547232,
            -0.00768756999018319,
            -0.00774092809890668,
            -0.00739843324341302,
            -0.00703374247295726
        ])
        
        # Speed grid
        self.v_grid = np.linspace(1, 5, 40)
        
        # Longitudinal PID control parameters
        self.s_pid_p = rospy.get_param('~s_pid_p', 0.5)
        self.s_pid_i = rospy.get_param('~s_pid_i', 0.1)
        self.s_pid_d = rospy.get_param('~s_pid_d', 0.0)
        self.s_error_sum = 0.0
        self.prev_s_error = 0.0
        self.max_s_error = rospy.get_param('~max_s_error', 1.0)
        
        # Switch to enable/disable longitudinal PID
        self.enable_longitudinal_pid = rospy.get_param('~enable_longitudinal_pid', True)
        
        # Subscribe topics
        rospy.Subscriber("odometry/filtered", Odometry, self.odom_callback)
        rospy.Subscriber("/estop", Bool, self.estop_callback)
        
        # Publish topics
        self.cmd_pub = rospy.Publisher("vel_and_steering", TwistStamped, queue_size=1)
        self.state_pub = rospy.Publisher("lateral_controller_state", Float64MultiArray, queue_size=10)
        self.estop_pub = rospy.Publisher("/estop", Bool, queue_size=1)
        self.cmd_ref_traj_pub = rospy.Publisher("cmd_ref_trajectory", RefTraj, queue_size=10)
        self.lateral_ref_pub = rospy.Publisher("lateral_ref", RefTraj, queue_size=10)
        self.nearest_idx_pub = rospy.Publisher("nearest_traj_idx", Int32, queue_size=10)
        
        # Load trajectory and initialize FIFO
        self.load_trajectory_file()
        self.initialize_fifo()
        
        # Control timer
        self.timer = rospy.Timer(rospy.Duration(1.0/self.control_rate), self.control_loop)
        
        # Set dynamic reconfiguration server
        self.dyn_server = Server(LongitudePIDParamsConfig, self.reconfigure_callback)
        
        rospy.loginfo("Lateral Controller initialized with trajectory management")
    
    def load_trajectory_file(self):
        """Load trajectory points from file"""
        try:
            if not os.path.exists(self.traj_file_path):
                rospy.logerr(f"Cannot open trajectory file: {self.traj_file_path}")
                return
            
            with open(self.traj_file_path, 'r') as file:
                for line in file:
                    if not line.strip():
                        continue
                    
                    values = line.strip().split()
                    if len(values) >= 7:
                        x, y, psi, v, curvature, s, d = map(float, values[:7])
                        
                        point = RefTraj()
                        point.header.stamp = rospy.Time.now()
                        point.header.frame_id = self.frame_id
                        point.x = x
                        point.y = y
                        point.psi = psi
                        point.v = v
                        point.curvature = curvature
                        point.s = s
                        point.d = d
                        
                        self.all_trajectory_points.append(point)
            
            rospy.loginfo(f"Successfully loaded {len(self.all_trajectory_points)} trajectory points")
        except Exception as e:
            rospy.logerr(f"Error loading trajectory file: {str(e)}")
    
    def initialize_fifo(self):
        """Initialize FIFO buffer with initial trajectory points"""
        # Clear FIFO
        self.fifo_buffer.clear()
        
        # Fill the first half+10 with zeros (velocity 0.6)
        for _ in range(self.fifo_size // 2 + 10):
            zero_point = RefTraj()
            zero_point.header.stamp = rospy.Time.now()
            zero_point.header.frame_id = self.frame_id
            zero_point.x = 0.0
            zero_point.y = 0.0
            zero_point.psi = 0.0
            zero_point.v = 0.6
            zero_point.curvature = 0.0
            zero_point.s = 0.0
            zero_point.d = 0.0
            
            self.fifo_buffer.append(zero_point)
        
        # Add initial trajectory points
        points_to_add = min(self.fifo_size // 2 - 10, len(self.all_trajectory_points))
        for i in range(points_to_add):
            self.fifo_buffer.append(self.all_trajectory_points[i])
        
        self.current_traj_idx = points_to_add
    
    def update_trajectory_buffer(self):
        """Update FIFO buffer with new trajectory points"""
        if not self.all_trajectory_points:
            return
        
        # Calculate middle index
        middle_idx = self.fifo_size // 2
        
        # Check if we've reached the end of trajectory
        if (self.current_traj_idx >= len(self.all_trajectory_points) and 
            len(self.fifo_buffer) > middle_idx and 
            self.fifo_buffer[middle_idx].x == self.all_trajectory_points[-1].x and 
            self.fifo_buffer[middle_idx].y == self.all_trajectory_points[-1].y):
            return
        
        # Remove points from the front
        for _ in range(min(10, len(self.fifo_buffer))):
            if self.fifo_buffer:
                self.fifo_buffer.popleft()
        
        # Add new trajectory points
        for _ in range(10):
            if self.current_traj_idx < len(self.all_trajectory_points):
                self.fifo_buffer.append(self.all_trajectory_points[self.current_traj_idx])
                self.current_traj_idx += 1
            else:
                self.fifo_buffer.append(self.all_trajectory_points[-1])
    
    def find_nearest_point(self):
        """Find the nearest trajectory point to current position"""
        if not self.fifo_buffer or not self.current_pose:
            return None, -1
        
        min_dist = float('inf')
        nearest_idx = 0
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        for i, point in enumerate(self.fifo_buffer):
            dx = point.x - current_x
            dy = point.y - current_y
            dist = dx * dx + dy * dy
            
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        return self.fifo_buffer[nearest_idx], nearest_idx
    
    def odom_callback(self, msg):
        """Handle odometry message"""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist.linear.x
        self.current_yaw_rate = msg.twist.twist.angular.z
        
        curr_yaw = self.get_quaternion_yaw(self.current_pose.orientation)
        
        # Global velocities
        vx_global = msg.twist.twist.linear.x * math.cos(curr_yaw)
        vy_global = msg.twist.twist.linear.x * math.sin(curr_yaw)
        
        # Get nearest point and transform velocities
        nearest_point, _ = self.find_nearest_point()
        if nearest_point:
            ref_heading = nearest_point.psi
            cos_ref = math.cos(ref_heading)
            sin_ref = math.sin(ref_heading)
            
            # Path-aligned velocities
            vx_path = vx_global * cos_ref + vy_global * sin_ref
            vy_path = -vx_global * sin_ref + vy_global * cos_ref
            
            self.y_dot = msg.twist.twist.linear.y
    
    def control_loop(self, event):
        """Main control loop"""
        if not self.fifo_buffer or not self.current_pose:
            rospy.logwarn_throttle(1.0, "Waiting for trajectory data or odometry")
            return
        
        try:
            # Update trajectory buffer
            self.update_trajectory_buffer()
            
            # Get middle point as command reference
            middle_idx = self.fifo_size // 2
            if len(self.fifo_buffer) > middle_idx:
                cmd_ref = self.fifo_buffer[middle_idx]
                self.cmd_ref_traj_pub.publish(cmd_ref)
            
            # Find nearest point
            nearest_point, nearest_idx = self.find_nearest_point()
            if nearest_point:
                self.lateral_ref_pub.publish(nearest_point)
                self.nearest_idx_pub.publish(Int32(nearest_idx))
            
            # Calculate control errors
            dx = self.current_pose.position.x - nearest_point.x
            dy = self.current_pose.position.y - nearest_point.y
            e_y = math.sqrt(dx*dx + dy*dy)
            
            # Determine error sign
            ref_heading = nearest_point.psi
            ref_dir_x = math.cos(ref_heading)
            ref_dir_y = math.sin(ref_heading)
            cross_product = -dx * ref_dir_y + dy * ref_dir_x
            e_y = e_y * (1 if cross_product >= 0 else -1)
            
            # Calculate heading error
            curr_yaw = self.get_quaternion_yaw(self.current_pose.orientation)
            e_psi = math.atan2(math.sin(curr_yaw - ref_heading), math.cos(curr_yaw - ref_heading))
            
            # Calculate curvature
            curvature = nearest_point.curvature
            
            # Calculate desired yaw rate
            dpsi_des = nearest_point.v * curvature
            
            # Calculate yaw rate error
            de_psi = self.current_yaw_rate - dpsi_des
            
            # Calculate lateral velocity error
            de_y = self.y_dot + nearest_point.v * e_psi
            
            # Calculate longitudinal position error (s error)
            s_error = cmd_ref.s - nearest_point.s
            
            # Use longitudinal PID or open-loop based on switch
            if self.enable_longitudinal_pid:
                # PID calculation
                s_error_deriv = (s_error - self.prev_s_error) * self.control_rate
                self.s_error_sum += s_error / self.control_rate
                
                # Integral limit
                self.s_error_sum = np.clip(self.s_error_sum, -1.0, 1.0)
                
                # Calculate velocity adjustment
                velocity_adjustment = (self.s_pid_p * s_error + 
                                     self.s_pid_i * self.s_error_sum + 
                                     self.s_pid_d * s_error_deriv)
                
                # Update previous error
                self.prev_s_error = s_error
                
                # Calculate final velocity command
                final_velocity = cmd_ref.v + velocity_adjustment
                final_velocity = max(0.0, final_velocity)  # Ensure velocity is not negative
                
                # If s error is too large, slow down or stop
                if abs(s_error) > self.max_s_error:
                    rospy.logwarn_throttle(1.0, f"S error too large {s_error:.2f}m, stop")
                    final_velocity = min(final_velocity, 0)
            else:
                # Use open-loop control (reference trajectory velocity directly)
                final_velocity = cmd_ref.v
                # Still record s_error for logging and debugging
                self.prev_s_error = s_error
            
            # Safety check
            if abs(e_y) > self.max_lateral_error or abs(e_psi) > math.radians(self.max_heading_error):
                rospy.logerr("Safety limits exceeded! e_y: %.2f, e_psi: %.2f", e_y, math.degrees(e_psi))
                # Only publish E-stop command if not already in E-stop state
                if not self.estop_active:
                    self.estop_active = True
                    self.safety_violation = True  # Mark as E-stop triggered by safety violation
                    self.estop_pub.publish(Bool(True))
                return
            
            # If state returns to safe range, clear safety violation flag
            if self.safety_violation and abs(e_y) < self.max_lateral_error*0.8 and abs(e_psi) < math.radians(self.max_heading_error*0.8):
                rospy.loginfo("System state returned to safe range, E-stop can be manually deactivated")
                self.safety_violation = False
            
            # State vector
            state = np.array([e_y, de_y, e_psi, de_psi])
            
            # Select gains based on current velocity
            idx = np.argmin(np.abs(self.v_grid - self.current_velocity))
            K_gain = self.K_gains[idx]
            L_gain = self.L_gains[idx]
            
            # Calculate steering command
            steer_cmd_rad = np.dot(K_gain, state) + L_gain * (curvature * self.wheelbase)
            
            # Limit steering angle
            steer_cmd_rad = np.clip(steer_cmd_rad, -self.max_steering_rad, self.max_steering_rad)
            
            # Check E-stop state
            if self.estop_active:
                # Do not send control commands, the chassis will automatically stop
                return

            # Publish command
            cmd_msg = TwistStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.header.frame_id = "base_link"
            cmd_msg.twist.linear.x = final_velocity  # Use PID adjusted velocity
            cmd_msg.twist.angular.z = steer_cmd_rad  # Steering angle (radians)
            self.cmd_pub.publish(cmd_msg)
            
            # Publish state information (for debugging)
            state_msg = Float64MultiArray()
            state_msg.data = [e_y, de_y, e_psi, de_psi, steer_cmd_rad, self.current_velocity, curvature, s_error, final_velocity, velocity_adjustment]
            self.state_pub.publish(state_msg)
            
            # Update previous steering angle
            self.prev_steering = steer_cmd_rad
            
        except Exception as e:
            rospy.logerr(f"Error in control loop: {str(e)}")
            if not self.estop_active:
                self.estop_active = True
                self.safety_violation = True
                self.estop_pub.publish(Bool(True))

    def estop_callback(self, msg):
        """Handle external E-stop command"""
        # If E-stop was triggered by safety violation, ignore deactivation requests
        if not msg.data and self.safety_violation:
            rospy.logwarn("Cannot deactivate E-STOP: System is in safety violation state, please restore safe state first")
            # Re-publish E-stop signal to ensure system remains in E-stop state
            self.estop_pub.publish(Bool(True))
            return
            
        # Only output logs when state changes
        if msg.data and not self.estop_active:
            self.estop_active = True
            rospy.logwarn("E-STOP ACTIVATED!")
        elif not msg.data and self.estop_active:
            self.estop_active = False
            self.safety_violation = False  # Clear safety violation flag when manually deactivating E-stop
            rospy.loginfo("E-STOP DEACTIVATED")
    
    def get_quaternion_yaw(self, q):
        """Extract yaw angle from quaternion"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def reconfigure_callback(self, config, level):
        """Dynamic reconfiguration parameter callback function"""
        # Update PID parameters
        self.s_pid_p = config.s_pid_p
        self.s_pid_i = config.s_pid_i
        self.s_pid_d = config.s_pid_d
        
        # Reset integral term to avoid integral saturation
        self.s_error_sum = 0.0
        
        rospy.loginfo("Lateral controller parameters updated: PID parameters p=%.2f, i=%.2f, d=%.2f", 
                     self.s_pid_p, self.s_pid_i, self.s_pid_d)
        return config

if __name__ == '__main__':
    try:
        controller = LateralController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 
