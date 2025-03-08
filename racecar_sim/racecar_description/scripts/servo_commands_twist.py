#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
import rospy
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from ackermann_msgs.msg import AckermannDriveStamped
 
 
L = 0.335 # 轴距，单位 m
B_f = 0.305 # 前轮轮距，单位 m
B_r = 0.305 # 后轮轮距，单位 m
max_rad = 0.7 # 最大转弯角度，单位 rad
 
 
# 设置最大的角度
def limsteer(data, maxdata):
    if data > 0 and data > maxdata:
        data = maxdata
    elif data < 0 and math.fabs(data) > maxdata:
        data = maxdata
    return data
 
 
# 接受 ackermann_msgs/AckermannDriveStamped 类型的消息 控制 阿克曼小车模型四个轮子的速度和前轮转角
def set_throttle_steer(data:TwistStamped):

    pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    throttle = data.twist.linear.x/0.047*2
    steer = data.twist.angular.z

    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(steer)
    pub_pos_right_steering_hinge.publish(steer)
 
    # 表格形式输出
    print(f"{'Wheel/Steering':<25}{'Value':<10}")
    print("-" * 35)
    print(f"{'Left Front Wheel Velocity : ':<25}{throttle:.2f} m/s")
    print(f"{'Right Front Wheel Velocity: ':<25}{throttle:.2f} m/s")
    print(f"{'Left Rear Wheel Velocity  : ' :<25}{throttle:.2f} m/s")
    print(f"{'Right Rear Wheel Velocity : ':<25}{throttle:.2f} m/s")
    print(f"{'Left Front Steering Angle : ':<25}{steer:.2f} rad")
    print(f"{'Right Front Steering Angle: ':<25}{steer:.2f} rad")
 
 
# 将 geometry_msgs/Twist 类型的消息 转换为 阿克曼小车模型四个轮子的速度和前轮转角
def set_speed(data:TwistStamped):
    global L, B_f, B_r, max_rad
    
    pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)
 
    pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)
    
    v = data.twist.linear.x  
    w = data.twist.angular.z
 
    if v != 0 and w != 0:
        # r 为理想的中间轮胎的转弯半径
        r = math.fabs(v/w) 
 
        # math.copysign(x, y)：返回一个与 x 数值大小相同、与 y 符号相同的浮点数
        rL_rear = r - (math.copysign(1, w) * (B_r/2.0)) 
        rR_rear = r + (math.copysign(1, w) * (B_r/2.0))
 
        L_front_temp = r - (math.copysign(1, w) * (B_f/2.0)) 
        R_front_temp = r + (math.copysign(1, w) * (B_f/2.0))
        rL_front = math.sqrt(math.pow(L_front_temp, 2) + math.pow(L, 2))
        rR_front = math.sqrt(math.pow(R_front_temp, 2) + math.pow(L, 2))
 
        # 速度, m/s
        vL_rear = v * rL_rear/r
        vR_rear = v * rR_rear/r
        vL_front = v * rL_front/r
        vR_front = v * rR_front/r
 
        # 角度, rad
        anL_front = math.atan2(L, L_front_temp) * math.copysign(1, w)
        anR_front = math.atan2(L, R_front_temp) * math.copysign(1, w)
 
    else:
        # 速度
        vL_rear = v
        vR_rear = v
        vL_front = v
        vR_front = v
 
        # 角度
        anL_front = w
        anR_front = w        
 
    anL_front = limsteer(anL_front, max_rad)  
    anR_front = limsteer(anR_front, max_rad)
   
    pub_vel_left_rear_wheel.publish(vL_rear)
    pub_vel_right_rear_wheel.publish(vR_rear)
    pub_vel_left_front_wheel.publish(vL_front)
    pub_vel_right_front_wheel.publish(vR_front)
    pub_pos_left_steering_hinge.publish(anL_front)
    pub_pos_right_steering_hinge.publish(anR_front)
 
    # 表格形式输出
    print(f"{'Wheel/Steering':<25}{'Value':<10}")
    print("-" * 35)
    print(f"{'Left Front Wheel Velocity : ':<25}{vL_front:.2f} m/s")
    print(f"{'Right Front Wheel Velocity: ':<25}{vR_front:.2f} m/s")
    print(f"{'Left Rear Wheel Velocity  : ' :<25}{vL_rear:.2f} m/s")
    print(f"{'Right Rear Wheel Velocity : ':<25}{vR_rear:.2f} m/s")
    print(f"{'Left Front Steering Angle : ':<25}{anL_front:.2f} rad")
    print(f"{'Right Front Steering Angle: ':<25}{anR_front:.2f} rad")
 
 
def servo_commands():
    rospy.init_node('servo_commands', anonymous=True)
 
    # rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, set_throttle_steer) 
    rospy.Subscriber("/vel_and_steering", TwistStamped, set_throttle_steer)
 
    rospy.spin()
 
if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass