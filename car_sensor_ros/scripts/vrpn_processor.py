#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_geometry_msgs import do_transform_pose

class VRPNProcessor:
    def __init__(self):
        rospy.init_node('vrpn_processor')
        
        # 配置参数
        self.world_frame = rospy.get_param('~world_frame', 'world')          
        self.odom_frame  = rospy.get_param('~odom_frame', 'odom')            
        self.car_frame   = rospy.get_param('~car_frame', 'base_footprint')    
        self.marker_topic = rospy.get_param('~marker_topic', '/vrpn_client_node/Marker/pose')
        self.car_topic = rospy.get_param('~car_topic', '/vrpn_client_node/RigidBody/pose')
        
        # 初始化标志物初始位置采集
        self.marker_positions = []           
        self.initial_marker_pose = None      
        
        # TF相关对象（分离静态/动态广播器）
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()  # ★★★静态变换广播器
        self.dynamic_tf_broadcaster = tf2_ros.TransformBroadcaster()       # ★★★动态变换广播器
        
        # 订阅动捕系统话题
        rospy.Subscriber(self.marker_topic, PoseStamped, self.marker_callback)
        rospy.Subscriber(self.car_topic, PoseStamped, self.car_callback)
        
        # 初始化其他参数
        self.sample_count = 0
        self.required_samples = rospy.get_param('~initial_samples', 10) 

        # 发布转换后的位姿话题
        self.pose_pub = rospy.Publisher('/vrpn_client_node/RigidBody/offset_pose', PoseStamped, queue_size=10)
        
    def marker_callback(self, msg):
        """处理标志物话题完成初始坐标系标定"""
        if self.initial_marker_pose is not None:
            return # 标定已完成
            
        self.marker_positions.append(msg.pose.position)
        self.sample_count += 1
        
        if self.sample_count >= self.required_samples:
            avg_x = np.mean([p.x for p in self.marker_positions])
            avg_y = np.mean([p.y for p in self.marker_positions])
            avg_z = np.mean([p.z for p in self.marker_positions])
            
            # 构建world到odom的静态变换
            self.initial_marker_pose = TransformStamped()
            self.initial_marker_pose.header.stamp = rospy.Time.now()
            self.initial_marker_pose.header.frame_id = self.world_frame
            self.initial_marker_pose.child_frame_id = self.odom_frame
            self.initial_marker_pose.transform.translation.x = avg_x
            self.initial_marker_pose.transform.translation.y = avg_y
            self.initial_marker_pose.transform.translation.z = avg_z
            self.initial_marker_pose.transform.rotation.w = 1.0  # 无旋转
            
            # ★★★ 使用静态广播器发送world->odom的恒定变换
            self.static_tf_broadcaster.sendTransform(self.initial_marker_pose)
            rospy.loginfo("初始化完成：odom坐标系已标定")
            
    def car_callback(self, msg):
        """处理小车位姿并发布动态odom->base_footprint变换"""
        if self.initial_marker_pose is None:
            rospy.logwarn_throttle(5.0, "等待标定完成...")  # 添加节流避免日志刷屏
            return
            
        try:
            # ★★★获取world到odom的静态变换
            static_transform = self.tf_buffer.lookup_transform(
                target_frame=self.odom_frame,
                source_frame=self.world_frame,
                time=rospy.Time(0),
                timeout=rospy.Duration(0.5)
            )
            
            # 转换小车位姿到odom坐标系
            car_pose_odom = do_transform_pose(msg, static_transform)
            
            # 发布转换后的位姿话题
            output_pose = PoseStamped()
            output_pose.header.stamp = msg.header.stamp
            output_pose.header.frame_id = self.odom_frame
            output_pose.pose = car_pose_odom.pose
            self.pose_pub.publish(output_pose)
            
            # ★★★构建并发布动态odom->base_footprint变换
            dynamic_tf = TransformStamped()
            dynamic_tf.header.stamp = msg.header.stamp  # 时间戳与原数据同步
            dynamic_tf.header.frame_id = self.odom_frame
            dynamic_tf.child_frame_id = self.car_frame
            dynamic_tf.transform.translation = output_pose.pose.position
            dynamic_tf.transform.rotation = output_pose.pose.orientation
            self.dynamic_tf_broadcaster.sendTransform(dynamic_tf)
            
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, f"坐标变换异常: {str(e)}")

if __name__ == '__main__':
    processor = VRPNProcessor()
    rospy.spin()
