#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseWithCovarianceStamped
from tf2_geometry_msgs import do_transform_pose

class VRPNProcessor:
    def __init__(self):
        rospy.init_node('vrpn_processor')
        
        # Configuration parameters
        self.world_frame = rospy.get_param('~world_frame', 'world')          
        self.odom_frame  = rospy.get_param('~odom_frame', 'odom')            
        self.car_frame   = rospy.get_param('~car_frame', 'base_footprint')    
        self.marker_topic = rospy.get_param('~marker_topic', '/vrpn_client_node/Marker/pose')
        self.car_topic = rospy.get_param('~car_topic', '/vrpn_client_node/RigidBody/pose')
        
        # Initialize marker initial position collection
        self.marker_positions = []           
        self.initial_marker_pose = None      

        # Initialize covariance matrix parameters
        self.covariance = self._load_covariance()
        
        # TF related objects (separate static/dynamic broadcasters)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()  # Static transform broadcaster
        self.dynamic_tf_broadcaster = tf2_ros.TransformBroadcaster()       # Dynamic transform broadcaster
        
        # Subscribe to motion capture system topics
        rospy.Subscriber(self.marker_topic, PoseStamped, self.marker_callback)
        rospy.Subscriber(self.car_topic, PoseStamped, self.car_callback)
        
        # Initialize other parameters
        self.sample_count = 0
        self.required_samples = rospy.get_param('~initial_samples', 10) 

        # Publish transformed pose topic
        self.pose_pub = rospy.Publisher('/vrpn_client_node/RigidBody/offset_pose', PoseWithCovarianceStamped, queue_size=10)

    def _load_covariance(self):
        """Load covariance configuration from parameter server"""
        covariance_diag = rospy.get_param('~covariance_diagonal',
                                         [0.1, 0.1, 0.1, 0.05, 0.05, 0.05])  # x,y,z,rot(x),rot(y),rot(z)
        
        # Construct 6x6 diagonal covariance matrix (row-major order)
        covariance = [0.0]*36
        indices = [0,7,14,21,28,35]  # Diagonal position indices
        for i, val in zip(indices, covariance_diag):
            covariance[i] = float(val)
            
        return covariance
        
    def marker_callback(self, msg):
        """Process marker topic to complete initial coordinate system calibration"""
        if self.initial_marker_pose is not None:
            return # Calibration completed
            
        self.marker_positions.append(msg.pose.position)
        self.sample_count += 1
        
        if self.sample_count >= self.required_samples:
            avg_x = np.mean([p.x for p in self.marker_positions])
            avg_y = np.mean([p.y for p in self.marker_positions])
            avg_z = np.mean([p.z for p in self.marker_positions])
            
            # Build static transform from world to odom
            self.initial_marker_pose = TransformStamped()
            self.initial_marker_pose.header.stamp = rospy.Time.now()
            self.initial_marker_pose.header.frame_id = self.world_frame
            self.initial_marker_pose.child_frame_id = self.odom_frame
            self.initial_marker_pose.transform.translation.x = avg_x
            self.initial_marker_pose.transform.translation.y = avg_y
            self.initial_marker_pose.transform.translation.z = avg_z
            self.initial_marker_pose.transform.rotation.w = 1.0  # No rotation
            
            # Use static broadcaster to send constant world->odom transform
            self.static_tf_broadcaster.sendTransform(self.initial_marker_pose)
            rospy.loginfo("Initialization complete: odom frame calibrated")
            
    def car_callback(self, msg):
        if self.initial_marker_pose is None: 
            return
            
        try:
            static_transform = self.tf_buffer.lookup_transform(
                target_frame=self.odom_frame,
                source_frame=self.world_frame,
                time=rospy.Time(0),
                timeout=rospy.Duration(0.5)
            )
            
            # Transform pose
            car_pose_odom = do_transform_pose(msg, static_transform)
            
            # Construct pose message with covariance
            output_pose = PoseWithCovarianceStamped()
            output_pose.header.stamp = msg.header.stamp           # Keep original timestamp
            output_pose.header.frame_id = self.odom_frame
            output_pose.pose.pose = car_pose_odom.pose
            output_pose.pose.covariance = self.covariance          # Add covariance
            
            self.pose_pub.publish(output_pose)
            
            # Build and publish dynamic odom->base_footprint transform
            # dynamic_tf = TransformStamped()
            # dynamic_tf.header.stamp = msg.header.stamp  # Synchronize timestamp with original data
            # dynamic_tf.header.frame_id = self.odom_frame
            # dynamic_tf.child_frame_id = self.car_frame
            # dynamic_tf.transform.translation = output_pose.pose.pose.position
            # dynamic_tf.transform.rotation = output_pose.pose.pose.orientation
            # self.dynamic_tf_broadcaster.sendTransform(dynamic_tf)
            
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, f"Transform exception: {str(e)}")

if __name__ == '__main__':
    processor = VRPNProcessor()
    rospy.spin()
