#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import scipy.io as sio
import os
import signal
import sys

class RosbagProcessor:
    def __init__(self):
        # 初始化节点
        rospy.init_node('rosbag_processor', anonymous=True)
        
        # 记录起始时间用于计算相对时间
        self.start_time = rospy.Time.now()
        
        # 数据存储
        self.lateral_controller_data = []
        self.lateral_controller_time = []
        
        self.odom_filtered_data = []
        self.odom_filtered_time = []
        
        self.ground_truth_data = []
        self.ground_truth_time = []
        
        self.vel_steering_data = []
        self.vel_steering_time = []
        
        # 话题订阅
        rospy.Subscriber('/lateral_controller_state', Float64MultiArray, self.lateral_controller_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_filtered_callback)
        rospy.Subscriber('/ground_truth/pose', Odometry, self.ground_truth_callback)
        rospy.Subscriber('/vel_and_steering', TwistStamped, self.vel_steering_callback)
        
        # 设置关闭节点时的回调函数
        rospy.on_shutdown(self.save_data_to_mat)
        
        # 注册信号处理函数，确保Ctrl+C能正确触发保存
        signal.signal(signal.SIGINT, self.signal_handler)
        
        rospy.loginfo("RosbagProcessor节点已初始化，正在记录数据...")

    def lateral_controller_callback(self, msg):
        # 记录数据
        data = np.array(msg.data)
        rel_time = (rospy.Time.now() - self.start_time).to_sec()
        
        self.lateral_controller_data.append(data)
        self.lateral_controller_time.append(rel_time)

    def odom_filtered_callback(self, msg):
        # 提取位置和姿态数据
        pose = msg.pose.pose
        twist = msg.twist.twist
        
        position = [pose.position.x, pose.position.y, pose.position.z]
        orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        linear_vel = [twist.linear.x, twist.linear.y, twist.linear.z]
        angular_vel = [twist.angular.x, twist.angular.y, twist.angular.z]
        
        data = position + orientation + linear_vel + angular_vel
        rel_time = (rospy.Time.now() - self.start_time).to_sec()
        
        self.odom_filtered_data.append(data)
        self.odom_filtered_time.append(rel_time)

    def ground_truth_callback(self, msg):
        # 提取位置和姿态数据
        pose = msg.pose.pose
        twist = msg.twist.twist
        
        position = [pose.position.x, pose.position.y, pose.position.z]
        orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        linear_vel = [twist.linear.x, twist.linear.y, twist.linear.z]
        angular_vel = [twist.angular.x, twist.angular.y, twist.angular.z]
        
        data = position + orientation + linear_vel + angular_vel
        rel_time = (rospy.Time.now() - self.start_time).to_sec()
        
        self.ground_truth_data.append(data)
        self.ground_truth_time.append(rel_time)

    def vel_steering_callback(self, msg):
        # 提取速度和舵角指令
        speed_cmd = msg.twist.linear.x
        steer_cmd = msg.twist.angular.z
        
        data = [speed_cmd, steer_cmd]
        rel_time = (rospy.Time.now() - self.start_time).to_sec()
        
        self.vel_steering_data.append(data)
        self.vel_steering_time.append(rel_time)

    def signal_handler(self, sig, frame):
        # 确保在Ctrl+C时保存数据
        rospy.loginfo("接收到中断信号，正在保存数据...")
        self.save_data_to_mat()
        sys.exit(0)

    def save_data_to_mat(self):
        rospy.loginfo("正在保存数据到MAT文件...")
        
        # 转换为numpy数组
        lateral_controller_data = np.array(self.lateral_controller_data) if self.lateral_controller_data else np.array([])
        lateral_controller_time = np.array(self.lateral_controller_time) if self.lateral_controller_time else np.array([])
        
        odom_filtered_data = np.array(self.odom_filtered_data) if self.odom_filtered_data else np.array([])
        odom_filtered_time = np.array(self.odom_filtered_time) if self.odom_filtered_time else np.array([])
        
        ground_truth_data = np.array(self.ground_truth_data) if self.ground_truth_data else np.array([])
        ground_truth_time = np.array(self.ground_truth_time) if self.ground_truth_time else np.array([])
        
        vel_steering_data = np.array(self.vel_steering_data) if self.vel_steering_data else np.array([])
        vel_steering_time = np.array(self.vel_steering_time) if self.vel_steering_time else np.array([])
        
        # 构建MATLAB字典
        mat_dict = {
            'lateral_controller_data': lateral_controller_data,
            'lateral_controller_time': lateral_controller_time,
            'lateral_controller_fields': ['e_y', 'de_y', 'e_psi', 'de_psi', 'steer_cmd_rad', 
                                         'current_velocity', 'curvature', 's_error', 
                                         'final_velocity', 'velocity_adjustment'],
            
            'odom_filtered_data': odom_filtered_data,
            'odom_filtered_time': odom_filtered_time,
            'odom_filtered_fields': ['pos_x', 'pos_y', 'pos_z', 'orient_x', 'orient_y', 
                                     'orient_z', 'orient_w', 'lin_vel_x', 'lin_vel_y', 
                                     'lin_vel_z', 'ang_vel_x', 'ang_vel_y', 'ang_vel_z'],
            
            'ground_truth_data': ground_truth_data,
            'ground_truth_time': ground_truth_time,
            'ground_truth_fields': ['pos_x', 'pos_y', 'pos_z', 'orient_x', 'orient_y', 
                                   'orient_z', 'orient_w', 'lin_vel_x', 'lin_vel_y', 
                                   'lin_vel_z', 'ang_vel_x', 'ang_vel_y', 'ang_vel_z'],
            
            'vel_steering_data': vel_steering_data,
            'vel_steering_time': vel_steering_time,
            'vel_steering_fields': ['speed_cmd', 'steer_cmd']
        }
        
        # 生成文件名
        timestamp = rospy.Time.now().to_sec()
        filename = f"rosbag_data_{timestamp:.0f}.mat"
        
        # 确保保存在用户的主目录
        home_dir = os.path.expanduser("~")
        filepath = os.path.join(home_dir, filename)
        
        # 保存文件
        sio.savemat(filepath, mat_dict)
        rospy.loginfo(f"数据已保存到文件: {filepath}")

def main():
    processor = RosbagProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("节点被用户终止")

if __name__ == '__main__':
    main()
