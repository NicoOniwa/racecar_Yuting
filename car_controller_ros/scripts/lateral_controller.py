#!/usr/bin/env python3

import rospy
import numpy as np
import math
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from traj_analyzer.msg import RefTraj
from std_msgs.msg import Float64MultiArray, Float64, Bool

class LateralController:
    def __init__(self):
        rospy.init_node('lateral_controller', anonymous=False)
        
        # Get parameters
        self.wheelbase = rospy.get_param('~wheelbase', 0.174)  # Vehicle wheelbase
        self.control_rate = rospy.get_param('~control_rate', 20.0)  # Control frequency
        self.max_steering_angle = rospy.get_param('~max_steering_angle', 25.0)  # Maximum steering angle (degrees)
        self.max_steering_rad = math.radians(self.max_steering_angle)
        
        # State variables
        self.cmd_ref_traj = None
        self.lateral_ref = None
        self.current_pose = None
        self.current_velocity = 0.0
        self.current_yaw_rate = 0.0
        self.y_dot = 0.0
        self.prev_steering = 0.0
        
        # E-stop state
        self.estop_active = False
        self.safety_violation = False  # Added: flag to indicate if E-stop was triggered by safety violation
        
        # Safety limits
        self.max_lateral_error = rospy.get_param('~max_lateral_error', 0.5)  # Maximum lateral error (meters)
        self.max_heading_error = rospy.get_param('~max_heading_error', 30.0)  # Maximum heading error (degrees)
        
        # Control gain matrix - obtained from MATLAB code
        self.K_gains = np.array([
            [-6.22364053081608, -0.119933555790367, -1.79306122252577, -0.0173770991784354],
            [-4.07314490642052, -0.0925069683909457, -1.45573836806723, -0.0183139479688961],
            [-2.94915232275821, -0.0767863912082127, -1.24793807284656, -0.0191143134089535],
            [-2.69930087707559, -0.0805767820749538, -1.21610148639450, -0.0211886450414383],
            [-1.81594001715976, -0.0588415569660538, -1.00027933749321, -0.0204913003317687],
            [-1.63234788170244, -0.0586635807318933, -0.965701976816002, -0.0218661763238577],
            [-1.24138350104978, -0.0476596756071716, -0.848596552384048, -0.0215325705556222],
            [-1.03469564034862, -0.0425673768734628, -0.784766670546516, -0.0218043412817321],
            [-0.941437906536761, -0.0418616869989587, -0.763221275423766, -0.0227590130943350],
            [-0.776294088429916, -0.0363029081988157, -0.700993493643848, -0.0226065394441894],
            [-0.622637501987140, -0.0301396865404903, -0.632858324721398, -0.0220320844083687],
            [-0.563151846560282, -0.0289017232266490, -0.613308787863649, -0.0225675660881218],
            [-0.512450256531042, -0.0278255569306132, -0.596812826812500, -0.0230955876141210],
            [-0.420472038829676, -0.0232933657057594, -0.545349653735793, -0.0224399005284455],
            [-0.387094021233674, -0.0226049330258396, -0.534710861896524, -0.0229503079326371],
            [-0.383414995799648, -0.0241762277727010, -0.549258566963019, -0.0242523562377475],
            [-0.308385665842894, -0.0194148064099780, -0.495515039330267, -0.0231549079859267],
            [-0.296791723284053, -0.0199389598342530, -0.500793743720041, -0.0240588787312930],
            [-0.256648425107808, -0.0175515076858973, -0.469613251983932, -0.0235181613625429],
            [-0.297908771317402, -0.0231730469279533, -0.512873552392554, -0.0254879860968071]
        ])
        
        self.L_gains = np.array([
            -0.129971119462624,
            -0.0869803873978075,
            -0.0634292121113410,
            -0.0530611004477516,
            -0.0368515532864838,
            -0.0271337162471516,
            -0.0203009086186435,
            -0.0192804387854424,
            -0.0172242281431122,
            -0.0145695174609380,
            -0.0105925170916231,
            -0.00991869968314029,
            -0.00760352531387266,
            -0.00484700287791326,
            -0.00232853575669480,
            -0.00348528454493122,
            -0.00155636903655431,
            -0.000595216137960191,
            0.00120042558630700,
            -0.00565267322383229
        ])
        
        # Speed grid
        self.v_grid = np.linspace(1, 5, 20)
        
        # Subscribe topics
        rospy.Subscriber("cmd_ref_trajectory", RefTraj, self.cmd_ref_callback)
        rospy.Subscriber("lateral_ref", RefTraj, self.lateral_ref_callback)
        rospy.Subscriber("odometry/filtered", Odometry, self.odom_callback)
        
        # Add E-stop subscriber
        rospy.Subscriber("/estop", Bool, self.estop_callback)
        
        # Publish topics
        self.cmd_pub = rospy.Publisher("vel_and_steering", TwistStamped, queue_size=10)
        self.state_pub = rospy.Publisher("lateral_controller_state", Float64MultiArray, queue_size=10)
        self.estop_pub = rospy.Publisher("/estop", Bool, queue_size=10)
        
        # Control timer
        self.timer = rospy.Timer(rospy.Duration(1.0/self.control_rate), self.control_loop)
        
        rospy.loginfo("Lateral Controller initialized")
    
    def cmd_ref_callback(self, msg):
        self.cmd_ref_traj = msg
    
    def lateral_ref_callback(self, msg):
        self.lateral_ref = msg
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        
        # Extract current velocity and yaw rate
        self.current_velocity = msg.twist.twist.linear.x
        self.current_yaw_rate = msg.twist.twist.angular.z
        
        curr_yaw = self.get_quaternion_yaw(self.current_pose.orientation)
        
        # Global velocities
        vx_global = msg.twist.twist.linear.x*math.cos(curr_yaw)
        vy_global = msg.twist.twist.linear.x*math.sin(curr_yaw)
        
        # If we have reference trajectory, transform to path-aligned coordinates
        if self.lateral_ref is not None:
            # Get reference heading (path direction)
            ref_heading = self.lateral_ref.psi   # ref is in radians
            
            # Transform global velocities to path-aligned coordinates
            # Rotation matrix from global to path-aligned frame
            cos_ref = math.cos(ref_heading)
            sin_ref = math.sin(ref_heading)
            
            # Path-aligned velocities (x along path, y perpendicular to path)
            vx_path = vx_global * cos_ref + vy_global * sin_ref
            vy_path = -vx_global * sin_ref + vy_global * cos_ref
            
            # Update lateral velocity
            self.y_dot = msg.twist.twist.linear.y
    
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
    
    def control_loop(self, event):
        # Check E-stop state
        if self.estop_active:
            # Do not send control commands, the chassis will automatically stop
            return
            
        if None in (self.cmd_ref_traj, self.lateral_ref, self.current_pose):
            # Detailed explanation of which messages are missing
            missing_msgs = []
            if self.cmd_ref_traj is None:
                missing_msgs.append("cmd_ref_trajectory")
            if self.lateral_ref is None:
                missing_msgs.append("lateral_ref")
            if self.current_pose is None:
                missing_msgs.append("odometry")
            
            rospy.logwarn_throttle(1.0, f"Waiting for messages: {', '.join(missing_msgs)}. The controller requires cmd_ref_trajectory, lateral_ref, and odometry messages to work properly.")
            return
        
        try:
            # Calculate lateral error e_y
            dx = self.current_pose.position.x - self.lateral_ref.x
            dy = self.current_pose.position.y - self.lateral_ref.y
            e_y = math.sqrt(dx*dx + dy*dy)
            
            # Determine error sign (left is positive, right is negative)
            # Calculate cross product of vector from reference point to current position with reference trajectory direction
            ref_heading = self.lateral_ref.psi   # ref is in radians
            ref_dir_x = math.cos(ref_heading)
            ref_dir_y = math.sin(ref_heading)
            cross_product = - dx * ref_dir_y + dy * ref_dir_x
            e_y = e_y * (1 if cross_product >= 0 else -1)
            
            # Calculate heading error e_psi
            curr_yaw = self.get_quaternion_yaw(self.current_pose.orientation)
            ref_yaw = self.lateral_ref.psi   # ref is in radians
            e_psi = math.atan2(math.sin(curr_yaw - ref_yaw), math.cos(curr_yaw - ref_yaw))
            
            # Calculate curvature
            curvature = self.lateral_ref.curvature
            
            # Calculate desired yaw rate
            dpsi_des = self.lateral_ref.v * curvature
            
            # Calculate yaw rate error
            de_psi = self.current_yaw_rate - dpsi_des
            
            # Calculate lateral velocity error
            de_y = self.y_dot + self.lateral_ref.v * e_psi
            
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
            
            # Publish command
            cmd_msg = TwistStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.header.frame_id = "base_link"
            cmd_msg.twist.linear.x = self.cmd_ref_traj.v  # Use reference trajectory velocity
            cmd_msg.twist.angular.z = steer_cmd_rad  # Steering angle (radians)
            self.cmd_pub.publish(cmd_msg)
            
            # Publish state information (for debugging)
            state_msg = Float64MultiArray()
            state_msg.data = [e_y, de_y, e_psi, de_psi, steer_cmd_rad, self.current_velocity, curvature]
            self.state_pub.publish(state_msg)
            
            # Update previous steering angle
            self.prev_steering = steer_cmd_rad
            
        except Exception as e:
            rospy.logerr("Error in control loop: %s", str(e))
            # Only publish E-stop command if not already in E-stop state
            if not self.estop_active:
                self.estop_active = True
                self.safety_violation = True  # Mark as E-stop triggered by safety violation
                self.estop_pub.publish(Bool(True))

if __name__ == '__main__':
    try:
        controller = LateralController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 
