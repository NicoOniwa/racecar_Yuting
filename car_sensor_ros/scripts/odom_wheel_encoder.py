#!/usr/bin/env python3
import rospy
import math
import tf
from math import tan, sin, cos
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance

class AckermannOdom:
    def __init__(self):
        rospy.init_node('ackermann_odom_node')
        
        # Key parameters initialization (adjust according to actual vehicle)
        self.wheelbase = rospy.get_param("~wheelbase", 0.174)      # Wheelbase (distance between front and rear axles)
        self.track_width = rospy.get_param("~track_width", 0.125) # Front track width
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.047/2) # Drive wheel radius
        
        # Subscribe to joint states (get actual speed and steering angle)
        self.joint_sub = rospy.Subscriber("/racecar/joint_states", JointState, self.joint_cb)
        # Publish odometry
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        # TF broadcaster (odom frame -> base_footprint)
        # self.tf_broadcaster = tf.TransformBroadcaster()
        
        # State variables
        self.x = 0.0     # Position x (meters)
        self.y = 0.0     # Position y 
        self.yaw = 0.0   # Yaw angle (radians)
        self.last_time = rospy.Time.now()
        
        # Store joint data
        self.steer_left_angle = 0.0   # Left front steering angle
        self.steer_right_angle = 0.0  # Right front steering angle
        self.wheel_left_vel = 0.0     # Left drive wheel velocity (rad/s)
        self.wheel_right_vel = 0.0    # Right drive wheel velocity
        rospy.spin()
    
    def joint_cb(self, msg):
        # Extract joint data
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            # Left front steering angle (adjust according to your URDF joint names)
            if name == "left_steering_joint":
                self.steer_left_angle = pos
            # Right front steering angle
            elif name == "right_steering_joint":
                self.steer_right_angle = pos
            # Left drive wheel velocity (e.g. left rear)
            elif name == "left_rear_axle":
                self.wheel_left_vel = vel
            # Right drive wheel velocity (right rear)
            elif name == "right_rear_axle":
                self.wheel_right_vel = vel
        
        # Calculate and publish odometry
        self.calculate_odom()
        
    def calculate_odom(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt <= 0.0:
            return
        
        #=== Ackermann kinematic model calculation start ===#
        # Calculate average steering angle (if left/right wheels have different angles due to mechanical constraints)
        steer_angle = (self.steer_left_angle + self.steer_right_angle) / 2.0
        
        # Calculate actual linear velocity of wheels (m/s)
        v_left = self.wheel_left_vel * self.wheel_radius
        v_right = self.wheel_right_vel * self.wheel_radius
        avg_speed = (v_left + v_right) / 2.0
        
        # Angular velocity (steering radius calculation for rear-wheel drive model)
        if abs(steer_angle) > 1e-3:  # When steering angle is not zero
            turn_radius = self.wheelbase / tan(steer_angle)
            angular_velocity = avg_speed / turn_radius
        else:                        # Zero angular velocity when going straight
            angular_velocity = 0.0
        
        # Decompose linear velocity to vehicle coordinate system
        linear_velocity_x = avg_speed * cos(self.yaw)
        linear_velocity_y = avg_speed * sin(self.yaw)
        
        #=== Update position (forward Euler integration) ===#
        self.x += linear_velocity_x * dt
        self.y += linear_velocity_y * dt
        self.yaw += angular_velocity * dt
        
        #=== Publish Odometry message ===#
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        
        # Position (note: no sideslip considered, y is typically 0)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation quaternion
        quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # Velocity
        odom.twist.twist.linear.x = avg_speed
        odom.twist.twist.angular.z = angular_velocity
        
        # Publish message
        self.odom_pub.publish(odom)
        
        #=== Broadcast TF ===#
        # self.tf_broadcaster.sendTransform(
        #     (self.x, self.y, 0),
        #     quat,
        #     current_time,
        #     "base_footprint",
        #     "odom"
        # )
        
        # Update time record
        self.last_time = current_time

if __name__ == '__main__':
    node = AckermannOdom()

