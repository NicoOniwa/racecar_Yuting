#!/usr/bin/env python3
import rospy
import numpy as np
import scipy.io as sio
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from tf.transformations import euler_from_quaternion

class RosbagDataProcessor:
    def __init__(self):
        # Initialize data storage containers
        self.reset_data()
        
        # Set output filename
        self.output_file = rospy.get_param("~output_file", "rosbag_data.mat")

        rospy.set_param('use_sim_time', True)
        self.reference_time = None  # Reference time base
        
        # Subscribe to four target topics
        rospy.Subscriber("/hardware_cmd", TwistStamped, self.cmd_callback)
        rospy.Subscriber("/vel_and_steering", TwistStamped, self.vel_callback)
        rospy.Subscriber("/odometry/filtered", Odometry, self.filtered_callback)
        rospy.Subscriber("/odom_encoder_imu", Odometry, self.encoder_imu_callback)
        
        # Register shutdown hook
        rospy.on_shutdown(self.save_data)
        
    def get_relative_time(self, stamp):
        """★★★Key Modification 2: Convert to relative time (using first data time as reference)★★★"""
        if self.reference_time is None:
            self.reference_time = stamp.to_sec() if stamp != rospy.Time(0) else rospy.get_time()
            return 0.0
        return (stamp.to_sec() - self.reference_time) if stamp != rospy.Time(0) else (rospy.get_time() - self.reference_time)
    
    def reset_data(self):
        """Initialize/reset data containers"""
        # TwistStamped data container [(t, x_vel, y_vel, z_vel, x_rot, y_rot, z_rot)]
        self.cmd_data = []
        self.vel_data = []
        
        # Odometry data container [(t, x, y, z, roll, pitch, yaw, x_vel, y_vel, z_vel, x_rot, y_rot, z_rot)]
        self.filtered_data = [] 
        self.encoder_imu_data = []
        
    def cmd_callback(self, msg):
        """Process /hardware_cmd"""
        t = self.get_relative_time(msg.header.stamp)
        twist = msg.twist
        self.cmd_data.append([
            t, 
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z
        ])
        
    def vel_callback(self, msg):
        """Process /vel_and_steering"""
        t = self.get_relative_time(msg.header.stamp)
        twist = msg.twist
        self.vel_data.append([
            t,
            twist.linear.x, twist.linear.y, twist.linear.z, 
            twist.angular.x, twist.angular.y, twist.angular.z
        ])
        
    def process_odom(self, msg):
        """Common logic for processing Odometry"""
        t = self.get_relative_time(msg.header.stamp)
        
        # Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Convert quaternion to Euler angles
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Velocities
        lin_vel = msg.twist.twist.linear
        ang_vel = msg.twist.twist.angular
        
        return [t, x, y, z, roll, pitch, yaw, 
                lin_vel.x, lin_vel.y, lin_vel.z, 
                ang_vel.x, ang_vel.y, ang_vel.z]
    
    def filtered_callback(self, msg):
        """Process /odometry/filtered"""
        self.filtered_data.append(self.process_odom(msg))
        
    def encoder_imu_callback(self, msg):
        """Process /odom_encoder_imu"""
        self.encoder_imu_data.append(self.process_odom(msg))
        
    def save_data(self):
        """Save data to MAT file"""
        # Convert lists to numpy arrays
        cmd_array = np.array(self.cmd_data).T if self.cmd_data else np.empty((7,0))
        vel_array = np.array(self.vel_data).T if self.vel_data else np.empty((7,0))
        
        filtered_array = np.array(self.filtered_data).T if self.filtered_data else np.empty((13,0))
        encoder_imu_array = np.array(self.encoder_imu_data).T if self.encoder_imu_data else np.empty((13,0))
        
        # Build MATLAB data structure
        mat_data = {
            # TwistStamped data (time as first row)
            "cmd_time": cmd_array[0,:] if cmd_array.size > 0 else np.empty(0),
            "cmd_data": cmd_array[1:7,:] if cmd_array.size > 0 else np.empty((6,0)),
            
            "vel_time": vel_array[0,:] if vel_array.size > 0 else np.empty(0),
            "vel_data": vel_array[1:7,:] if vel_array.size > 0 else np.empty((6,0)),
            
            # Odometry data (time as first column)
            "filtered_time": filtered_array[0,:] if filtered_array.size > 0 else np.empty(0),
            "filtered_data": filtered_array[1:13,:] if filtered_array.size > 0 else np.empty((12,0)),
            
            "encoder_imu_time": encoder_imu_array[0,:] if encoder_imu_array.size > 0 else np.empty(0),
            "encoder_imu_data": encoder_imu_array[1:13,:] if encoder_imu_array.size > 0 else np.empty((12,0))
        }
        
        # Save file
        sio.savemat(self.output_file, mat_data)
        rospy.loginfo(f"Data saved to {self.output_file}")
        
        # Reset data
        self.reset_data()

if __name__ == '__main__':
    rospy.init_node('rosbag_data_processor')
    processor = RosbagDataProcessor()
    rospy.spin()
