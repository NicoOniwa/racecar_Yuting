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
        
        # 关键参数初始化（需根据实际车辆调整）
        self.wheelbase = rospy.get_param("~wheelbase", 0.174)      # 轴距，前后轮距离
        self.track_width = rospy.get_param("~track_width", 0.125) # 前轮轮距
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.047/2) # 驱动轮半径
        
        # 订阅关节状态（获取实际速度和转向角）
        self.joint_sub = rospy.Subscriber("/racecar/joint_states", JointState, self.joint_cb)
        # 发布里程计
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        # TF广播器（odom坐标系->base_footprint）
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # 状态变量
        self.x = 0.0     # 位置x (meters)
        self.y = 0.0     # 位置y 
        self.yaw = 0.0   # 航向角 (radians)
        self.last_time = rospy.Time.now()
        
        # 保存关节数据
        self.steer_left_angle = 0.0   # 左前轮转向角
        self.steer_right_angle = 0.0  # 右前轮转向角
        self.wheel_left_vel = 0.0     # 左驱动轮转速（rad/s）
        self.wheel_right_vel = 0.0    # 右驱动轮转速
        rospy.spin()
    
    def joint_cb(self, msg):
        # 提取各关节的数据
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            # 左前轮转向角（根据你的URDF关节名修改）
            if name == "left_steering_joint":
                self.steer_left_angle = pos
            # 右前轮转向角
            elif name == "right_steering_joint":
                self.steer_right_angle = pos
            # 左驱动轮速度（例如左后轮）
            elif name == "left_rear_axle":
                self.wheel_left_vel = vel
            # 右驱动轮速度（右后轮）
            elif name == "right_rear_axle":
                self.wheel_right_vel = vel
        
        # 计算并发布里程计
        self.calculate_odom()
        
    def calculate_odom(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt <= 0.0:
            return
        
        #=== 阿克曼运动学模型计算开始 ===#
        # 计算平均转向角（如果左右轮因机械限制转向角不同）
        steer_angle = (self.steer_left_angle + self.steer_right_angle) / 2.0
        
        # 计算左右轮的实际线速度（m/s）
        v_left = self.wheel_left_vel * self.wheel_radius
        v_right = self.wheel_right_vel * self.wheel_radius
        avg_speed = (v_left + v_right) / 2.0
        
        # 角速度（后轮驱动模型的转向半径计算）
        if abs(steer_angle) > 1e-3:  # 转向角不为零时
            turn_radius = self.wheelbase / tan(steer_angle)
            angular_velocity = avg_speed / turn_radius
        else:                        # 直行时角速度为0
            angular_velocity = 0.0
        
        # 线速度分解到车身坐标系
        linear_velocity_x = avg_speed * cos(self.yaw)
        linear_velocity_y = avg_speed * sin(self.yaw)
        
        #=== 更新位置（前向欧拉积分）===#
        self.x += linear_velocity_x * dt
        self.y += linear_velocity_y * dt
        self.yaw += angular_velocity * dt
        
        #=== 发布Odometry消息 ===#
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        
        # 位置（注意：未考虑侧滑，y通常为0）
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # 姿态四元数
        quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # 速度
        odom.twist.twist.linear.x = avg_speed
        odom.twist.twist.angular.z = angular_velocity
        
        # 发布消息
        self.odom_pub.publish(odom)
        
        #=== 广播TF ===#
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0),
            quat,
            current_time,
            "base_footprint",
            "odom"
        )
        
        # 更新时间记录
        self.last_time = current_time

if __name__ == '__main__':
    node = AckermannOdom()

