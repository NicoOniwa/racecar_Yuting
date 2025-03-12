#!/usr/bin/env python3

#import sys
#sys.path.append("/usr/local/lib/python3.6/dist-packages/jetracer-0.0.0-py3.6.egg")

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
import math
from jetracer.nvidia_racecar import NvidiaRacecar
import time
from dynamic_reconfigure.server import Server
from thesis_car_drive.cfg import ChassisParamsConfig

ESC_Init_Value = [1.1503, 0.18, 0.4]  # Updated throttle gain to 0.4
CMD_TIMEOUT_VALUE = 0.5   # if no command receive in 0.5s, the car will stop
MAXVEL_MAPPING_THROTTLE = 5.0      # Updated max velocity to 5.0
DEFAULT_MAX_STEERING_ANGLE = 25.0  # Default max steering angle is ±25 degrees 

# PID controller parameters
DEFAULT_KP = 0.1  # Proportional gain
DEFAULT_KI = 0.0  # Integral gain
DEFAULT_KD = 0.0  # Derivative gain

# velocity-throttle relationship model parameters
DEFAULT_THROTTLE_SLOPE = 18.2  # velocity-throttle relationship slope
DEFAULT_THROTTLE_OFFSET = 2.17  # velocity-throttle relationship offset
DEFAULT_MIN_VELOCITY = 0.5  # Minimum velocity threshold

class JetRacer_Chassis_Node:
    def __init__(self):
        # Publishers for raw control commands 
        # steering_cmd = steering_gain * angular.z + steering offset
        # throttle_cmd = throttle_gain * linear.x
        self.hardware_cmd_pub = rospy.Publisher("/hardware_cmd", TwistStamped, queue_size=10)
        self.hardware_cmd_stamped_msg = TwistStamped()

        # Add new publisher for actual steering angle
        self.actual_steering_pub = rospy.Publisher("/actual_steering", Float64, queue_size=1)

        rospy.init_node("jetracer_chassis", anonymous = False)

        # get the init value for controlling the ESC, default value is ESC_Init_Value
        self.hardware_init_param = rospy.get_param('~ESC_init_value',ESC_Init_Value)
        rospy.loginfo(f'Parameter steering gain is {self.hardware_init_param[0]}, steering offset is {self.hardware_init_param[1]}, throttle gain is {self.hardware_init_param[2]}.')

        print("Hardware Initiating...")
        self.HW_car = NvidiaRacecar()
        print("Hardware Initiating Finished!")
        rospy.sleep(0.5)
        # set steering_gain and steering offset
        self.HW_car.steering_gain = self.hardware_init_param[0]
        rospy.sleep(0.5)
        self.steering_gain_set = self.HW_car.steering_gain
        rospy.sleep(0.5)
        self.HW_car.steering_offset = self.hardware_init_param[1]
        rospy.sleep(0.5)
        self.steering_offset_set = self.HW_car.steering_offset
        rospy.sleep(0.1)
        print(f"steering gain is {self.HW_car.steering_gain}.")
        print(f"steering offset is {self.HW_car.steering_offset}")

        self.HW_car.steering = 0.2
        rospy.sleep(0.1)
        self.HW_car.steering = 0.0
        print("Now the actual steering angle should be zero!")

        # set throttle to zero
        self.HW_car.throttle = 0.0
        rospy.sleep(0.2)
        self.HW_car.throttle_gain = self.hardware_init_param[2]
        rospy.sleep(0.2)
        self.throttle_gain_set = self.HW_car.throttle_gain
        print(f"throttle gain is {self.HW_car.throttle_gain}.")

        self.gain_corresponding_vel_max = rospy.get_param('~maxvel_mapping_throttle',MAXVEL_MAPPING_THROTTLE)

        # Get max steering angle parameter (in degrees, convert to radians)
        max_steering_deg = rospy.get_param('~max_steering_angle', DEFAULT_MAX_STEERING_ANGLE)  # Default 25 degrees
        self.max_steering_angle = math.radians(max_steering_deg)
        rospy.loginfo(f'Max steering angle set to {max_steering_deg} degrees ({self.max_steering_angle:.4f} radians)')

        self.last_msg_time = time.time()
        
        # init vel and steer
        self.vel_x = 0.0
        self.steering_angle = 0.0

        # Get PID parameters
        self.kp = rospy.get_param('~pid_kp', DEFAULT_KP)
        self.ki = rospy.get_param('~pid_ki', DEFAULT_KI)
        self.kd = rospy.get_param('~pid_kd', DEFAULT_KD)
        
        # PID controller variables
        self.error_sum = 0.0
        self.last_error = 0.0
        self.target_velocity = 0.0
        self.actual_velocity = 0.0
        self.velocity_estimate = [0.0, 0.0, 0.0, 0.0]  # Initialize with zeros

        # velocity-throttle relationship model parameters
        self.throttle_slope = rospy.get_param('~throttle_slope', DEFAULT_THROTTLE_SLOPE)
        self.throttle_offset = rospy.get_param('~throttle_offset', DEFAULT_THROTTLE_OFFSET)
        self.min_velocity = rospy.get_param('~min_velocity', DEFAULT_MIN_VELOCITY)

        # Add velocity feedback publisher
        self.velocity_feedback_pub = rospy.Publisher("/velocity_feedback", Float64, queue_size=1)
        
        # Add PID output publisher (for debugging)
        self.pid_output_pub = rospy.Publisher("/pid_output", Float64, queue_size=1)
        
        # Add feedforward output publisher (for debugging)
        self.feedforward_pub = rospy.Publisher("/feedforward_output", Float64, queue_size=1)
        
        rospy.loginfo(f'PID parameters: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}')

        # set dynamic reconfiguration server
        self.dyn_server = Server(ChassisParamsConfig, self.reconfigure_callback)

        # set timeout (default is CMD_TIMEOUT_VALUE)
        self.timeout = rospy.get_param('~Cmd_Timeout_Value',CMD_TIMEOUT_VALUE)

        self.timer = rospy.Timer(rospy.Duration(1.0/50), self.control_loop)  # 50Hz

        # Subscribe to hall sensor topics
        self.encoder_sub = rospy.Subscriber("/encoder", Float32MultiArray, self.encoder_callback)
        self.vel_est_sub = rospy.Subscriber("/vel_est", Float32MultiArray, self.vel_est_callback)

        # Subscribe to move commands, Twist.linear.x stands for velocity
        # Twist.angular.z stands for steering angle
        self.move_cmd_sub = rospy.Subscriber("/vel_and_steering", TwistStamped, self.move_cmd_callback)


    def move_cmd_callback(self, msg):
        # Special case for zero velocity (stop command)
        if abs(msg.twist.linear.x) < 0.01:
            self.target_velocity = 0.0
        # Apply minimum velocity threshold for non-zero but low velocities
        elif abs(msg.twist.linear.x) < self.min_velocity:
            self.target_velocity = self.min_velocity if msg.twist.linear.x > 0 else -self.min_velocity
        else:
            self.target_velocity = msg.twist.linear.x
        
        # Limit steering angle to the configured max range
        self.steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, msg.twist.angular.z))
        self.last_msg_time = time.time()

    def control_loop(self, event):
        if time.time() - self.last_msg_time > self.timeout:
            self.target_velocity = 0.0
            self.steering_angle = 0.0
            self.error_sum = 0.0  # Reset integral term
            self.last_error = 0.0  # Reset derivative term
            self.send_motor_command(self.target_velocity, self.steering_angle)
        else:
            # Use PID controller to calculate throttle command
            throttle_cmd = self.calculate_model_based_control()
            self.send_motor_command_with_throttle(throttle_cmd, self.steering_angle)

    def calculate_model_based_control(self):
        # Model-based feedforward control
        if abs(self.target_velocity) < 0.01:
            # Target velocity close to zero, return zero control
            feedforward = 0.0
        else:
            # Calculate required throttle based on inverse model
            # From vel_est = throttle * 18.2 - 2.17
            # We get throttle = (vel_est + 2.17) / 18.2
            feedforward = (self.target_velocity + self.throttle_offset) / self.throttle_slope/self.throttle_gain_set
        
        # PID part for correcting model errors
        error = self.target_velocity - self.actual_velocity
        
        # Calculate PID terms
        p_term = self.kp * error
        
        self.error_sum += error
        max_sum = 0.5  # Limit the maximum value of integral term
        self.error_sum = max(-max_sum, min(max_sum, self.error_sum))
        i_term = self.ki * self.error_sum
        
        error_diff = error - self.last_error
        self.last_error = error
        d_term = self.kd * error_diff
        
        pid_correction = p_term + i_term + d_term
        
        # Combine feedforward and PID correction
        final_output = feedforward + pid_correction
        
        # Limit output range
        final_output = max(-1.0, min(1.0, final_output))
        
        # Publish debug information
        self.pid_output_pub.publish(pid_correction)
        self.feedforward_pub.publish(feedforward)  # Need to add a new publisher
        
        return final_output

    def send_motor_command_with_throttle(self, throttle_cmd, steering_angle):
        # Directly use calculated throttle command
        throttle_send = max(-1, min(1, throttle_cmd))
        self.HW_car.throttle = throttle_send
        
        # Ensure steering angle is within limits
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
        # Send command to ESC
        self.HW_car.steering = steering_angle

        # Build publish message
        self.hardware_cmd_stamped_msg.header.stamp = rospy.Time.now()
        self.hardware_cmd_stamped_msg.header.frame_id = "car_frame"

        # Calculate actual hardware command value, ensuring it's within hardware limits
        steering_cmd = self.steering_gain_set*steering_angle + self.steering_offset_set
        steering_cmd = max(-1, min(1, steering_cmd))  # Ensure hardware command is within [-1,1] range
        
        self.hardware_cmd_stamped_msg.twist.angular.z = steering_cmd
        self.hardware_cmd_stamped_msg.twist.linear.x = self.throttle_gain_set*throttle_send

        self.hardware_cmd_pub.publish(self.hardware_cmd_stamped_msg)

        # Publish actual steering angle (in degrees for easier visualization)
        self.actual_steering_pub.publish(steering_angle*180.0/math.pi)

    def send_motor_command(self, velocity, steering_angle):
        # Keep original open-loop control method for initialization and emergency stop
        throttle_send = max(-1, min(1, velocity/self.gain_corresponding_vel_max))
        self.HW_car.throttle = throttle_send
        
        # Ensure steering angle is within limits
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
        # Send command to ESC
        self.HW_car.steering = steering_angle

        # Build publish message
        self.hardware_cmd_stamped_msg.header.stamp = rospy.Time.now()
        self.hardware_cmd_stamped_msg.header.frame_id = "car_frame"

        # Calculate actual hardware command value, ensuring it's within hardware limits
        steering_cmd = self.steering_gain_set*steering_angle + self.steering_offset_set
        steering_cmd = max(-1, min(1, steering_cmd))  # Ensure hardware command is within [-1,1] range
        
        self.hardware_cmd_stamped_msg.twist.angular.z = steering_cmd
        self.hardware_cmd_stamped_msg.twist.linear.x = self.throttle_gain_set*throttle_send

        self.hardware_cmd_pub.publish(self.hardware_cmd_stamped_msg)

        # Publish actual steering angle (in degrees for easier visualization)
        self.actual_steering_pub.publish(steering_angle*180.0/math.pi)

    def cleanup(self):
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
            rospy.signal_shutdown("Chassis Node exited.")

    # Callback functions for hall sensor data
    def encoder_callback(self, msg):
        # Process encoder data from all wheels
        self.encoder_data = msg.data
        
    def vel_est_callback(self, msg):
        try:
            # Process velocity estimation data from all wheels
            self.velocity_estimate = msg.data
            if len(self.velocity_estimate) >= 4:
                # Calculate rear wheel average speed (BL + BR)/2
                self.actual_velocity = (self.velocity_estimate[2] + self.velocity_estimate[3]) / 2.0
                # Publish actual velocity feedback
                self.velocity_feedback_pub.publish(self.actual_velocity)
        except Exception as e:
            rospy.logerr(f"Error in velocity estimation callback: {e}")

    def reconfigure_callback(self, config, level):
        # Update PID parameters
        self.kp = config.pid_kp
        self.ki = config.pid_ki
        self.kd = config.pid_kd
        
        # Update throttle gain
        self.HW_car.throttle_gain = config.throttle_gain
        rospy.sleep(0.1)
        self.throttle_gain_set = self.HW_car.throttle_gain
        
        # Update model parameters
        self.throttle_slope = config.throttle_slope
        self.throttle_offset = config.throttle_offset
        self.min_velocity = config.min_velocity
        
        # Log parameter changes
        rospy.loginfo(f"PID parameters updated: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
        rospy.loginfo(f"Throttle gain updated: {self.throttle_gain_set}")
        rospy.loginfo(f"Model parameters updated: slope={self.throttle_slope}, offset={self.throttle_offset}")
        rospy.loginfo(f"Minimum velocity threshold updated: {self.min_velocity} m/s")
    
        # Reset PID controller state
        self.error_sum = 0.0
        self.last_error = 0.0
        
        return config

if __name__ == "__main__":
    chassis = JetRacer_Chassis_Node()
    chassis.run()
