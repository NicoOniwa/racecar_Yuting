#!/usr/bin/env python3

#import sys
#sys.path.append("/usr/local/lib/python3.6/dist-packages/jetracer-0.0.0-py3.6.egg")

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
import math
from jetracer.nvidia_racecar import NvidiaRacecar
import time

ESC_Init_Value = [0.82,0.18,0.3]  # steering gain, steering offset, and throttle gain
CMD_TIMEOUT_VALUE = 0.5   # if no command receive in 0.5s,the car will stop
MAXVEL_MAPPING_THROTTLE = 1      # the default max velocity for the specified throttle gain

class JetRacer_Chassis_Node:
    def __init__(self):
        # Subscribe to move commands, Twist.linear.x stands for velocity
        # Twist.angular.z stands for steering angle
        self.move_cmd_sub = rospy.Subscriber("/vel_and_steering", TwistStamped, self.move_cmd_callback)

        # Publishers for raw control commands 
        # steering_cmd = steering_gain * angular.z + steering offset
        # throttle_cmd = throttle_gain * linear.x
        self.hardware_cmd_pub = rospy.Publisher("/hardware_cmd", TwistStamped, queue_size=10)
        self.twist_stamped_msg = TwistStamped()

        rospy.init_node("jetracer_chassis", anonymous = False)

        # get the init value for controlling the ESC, default value is ESC_Init_Value
        self.hardware_init_param = rospy.get_param('ESC_init_value',ESC_Init_Value)
        rospy.loginfo(f'Parameter steering gain is {self.hardware_init_param[0]}, steering offset is {self.hardware_init_param[1]}, throttle gain is {self.hardware_init_param[2]}.')

        print("Hardware Initiating...")
        self.HW_car = NvidiaRacecar()
        print("Hardware Initiating Finished!")
        # set steering_gain and steering offset
        self.HW_car.steering_gain = self.hardware_init_param[0]
        self.steering_gain_set = self.HW_car.steering_gain
        rospy.sleep(0.1)
        self.HW_car.steering_offset = self.hardware_init_param[1]
        self.steering_offset_set = self.HW_car.steering_offset
        rospy.sleep(0.1)
        print(f"steering gain is {self.HW_car.steering_gain}.")
        print(f"steering offset is {self.HW_car.steering_offset}")

        self.HW_car.steering = 0.0
        print("Now the actual steering angle should be zero!")

        # set throttle to zero
        self.HW_car.throttle = 0.0
        self.HW_car.throttle_gain = self.hardware_init_param[2]
        self.throttle_gain_set = self.HW_car.throttle_gain
        print(f"throttle gain is {self.HW_car.throttle_gain}.")

        self.gain_corresponding_vel_max = rospy.get_param('maxvel_mapping_throttle',MAXVEL_MAPPING_THROTTLE)

        self.last_msg_time = time.time()
        
        # init vel and steer
        self.vel_x = 0.0
        self.steering_angle = 0.0

        # set timeout (default is CMD_TIMEOUT_VALUE)
        self.timeout = rospy.get_param('Cmd_Timeout_Value',CMD_TIMEOUT_VALUE)

        self.timer = rospy.Timer(rospy.Duration(1.0/50), self.control_loop)  # 50Hz

    def move_cmd_callback(self, msg):
        self.vel_x = msg.twist.linear.x
        self.steering_angle = msg.twist.angular.z
        self.last_msg_time = time.time()

    def control_loop(self, event):
        if time.time() - self.last_msg_time > self.timeout:
            self.vel_x = 0.0
            self.steering_angle = 0.0
            self.send_motor_command(self.vel_x, self.steering_angle)
        else:
            self.send_motor_command(self.vel_x, self.steering_angle)

    def send_motor_command(self,velocity,steering_angle):
        # the command from /vel_and_steering is in m/s and rad
        # should map into the hardware value between [-1,1]
        throttle_send = max(-1,min(1,velocity/self.gain_corresponding_vel_max))
        
        # send commond to ESC
        self.HW_car.throttle = throttle_send
        self.HW_car.steering = steering_angle

        # formulate the publish topic
        self.twist_stamped_msg.header.stamp = rospy.Time.now()
        self.twist_stamped_msg.header.frame_id = "car_frame"

        self.twist_stamped_msg.twist.angular.z = self.steering_gain_set*steering_angle + self.steering_offset_set
        self.twist_stamped_msg.twist.linear.x = self.throttle_gain_set*throttle_send

        self.hardware_cmd_pub.publish(self.twist_stamped_msg)

    def cleanup():
        rospy.loginfo("Shutting down... Stopping motor.")
        self.send_motor_command(0.0,0.0)
        rospy.sleep(0.3)
        rospy.loginfo("Motor Stop Cleanly.")

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.logwarn("Received Ctrl+C, shutting down...")
            self.cleanup()
        finally:
            rospy.signal_shutdown("Chasis Node exited.")


if __name__ == "__main__":
    chassis = JetRacer_Chassis_Node()
    chassis.run()
