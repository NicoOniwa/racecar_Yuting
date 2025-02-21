#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

class EncoderToJointState:
    def __init__(self):
        rospy.init_node('encoders_to_joint_state_node')
        
        # 参数配置
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.0235)    # 轮胎半径 (m)
        self.encoder_resolution = rospy.get_param("~encoder_resolution", 4)  # 编码器每转脉冲数
        
        # 四个驱动轮的关节命名（与URDF中的关节名必须一致）
        self.joint_names = [
            "left_front_axle",
            "right_front_axle",
            "left_rear_axle",
            "right_rear_axle"
        ]
        
        # 初始化数据存储
        self.last_encoder_counts = [0, 0, 0, 0]  # 每个轮子的最新脉冲计数 [FL, FR, BL, BR]
        self.last_vel_est = [0.0, 0.0, 0.0, 0.0] # 每个轮子的最新线速度估计（m/s）
        
        # 订阅原始数据
        rospy.Subscriber("/encoder", Float32MultiArray, self.encoder_cb)
        rospy.Subscriber("/vel_est", Float32MultiArray, self.vel_est_cb)
        
        # 发布转换后的JointState
        self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        
    def encoder_cb(self, msg):
        # 提取四轮脉冲计数 [FL, FR, BL, BR]
        counts = msg.data  
        if len(counts) != 4:
            rospy.logwarn("无效的encoder数据长度")
            return
        
        # 根据脉冲计数计算角度（单位：弧度）
        # 单个脉冲对应弧度：2π / encoder_resolution
        rad_per_pulse = (2 * 3.141592653589793) / self.encoder_resolution
        positions = [count * rad_per_pulse for count in counts]
        self.last_encoder_counts = positions  # 存储最新位置
        
    def vel_est_cb(self, msg):
        # 提取四轮线速度估计（m/s）
        velocities = msg.data
        if len(velocities) != 4:
            rospy.logwarn("无效的vel_est数据长度")
            return
        
        # 将线速度转换为角速度（rad/s）: ω = v / r
        angular_velocities = [v / self.wheel_radius for v in velocities]
        self.last_vel_est = angular_velocities
        
        # 每次收到速度更新时发布最新的JointState
        self.publish_joint_state()
        
    def publish_joint_state(self):
        # 构建JointState消息
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_names
        joint_state.position = self.last_encoder_counts
        joint_state.velocity = self.last_vel_est  # 角速度（rad/s）
        
        # 发布消息
        self.joint_pub.publish(joint_state)
        
if __name__ == '__main__':
    node = EncoderToJointState()
    rospy.spin()
