#!/usr/bin/env python3
import rospy
import tf2_ros
from sensor_msgs.msg import Imu
from geometry_msgs.msg import (
    PoseStamped,    # 📌 新增消息类型
    TwistStamped,
    AccelStamped,
    Vector3, 
    Quaternion
)
from tf2_ros import Buffer, TransformListener
from tf.transformations import quaternion_multiply, quaternion_inverse

class ImuTransformer:
    def __init__(self):
        rospy.init_node('imu_transformer_node', anonymous=True)
        
        # TF 监听初始化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        
        # 订阅原始IMU数据
        self.imu_sub = rospy.Subscriber("/imu_data", Imu, self.imu_callback)
        
        # 发布器们
        self.imu_pub = rospy.Publisher("/imu_data_base_link", Imu, queue_size=10)
        self.twist_pub = rospy.Publisher("/base_link/angular_velocity", TwistStamped, queue_size=10)
        self.accel_pub = rospy.Publisher("/base_link/linear_acceleration", AccelStamped, queue_size=10)
        self.pose_pub = rospy.Publisher("/base_link/orientation", PoseStamped, queue_size=10)  # 📌 新增发布器
        rospy.spin()

    def imu_callback(self, raw_imu):
        try:
            # 获取坐标变换
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                raw_imu.header.frame_id,
                rospy.Time(0),
                timeout=rospy.Duration(0.1)
            )
            
            # 转换后的IMU消息
            imu_transformed = Imu()
            imu_transformed.header.stamp = rospy.Time.now()
            imu_transformed.header.frame_id = "base_link"
            
            # 姿态四元数转换
            q_imu_to_base = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            q_base_to_imu = quaternion_inverse(q_imu_to_base)
            q_transformed = quaternion_multiply(
                [raw_imu.orientation.x, raw_imu.orientation.y, raw_imu.orientation.z, raw_imu.orientation.w],
                q_base_to_imu
            )
            imu_transformed.orientation = Quaternion(*q_transformed)
            
            # 注意：这里假设角速度和线加速度无需转换，若需要请参考之前讨论的旋转逻辑
            imu_transformed.angular_velocity = raw_imu.angular_velocity  
            imu_transformed.linear_acceleration = raw_imu.linear_acceleration  
            
            # 发布转换后的IMU
            self.imu_pub.publish(imu_transformed)
            
            # 发布角速度（TwistStamped）
            twist_msg = TwistStamped()
            twist_msg.header = imu_transformed.header
            twist_msg.twist.angular = imu_transformed.angular_velocity
            self.twist_pub.publish(twist_msg)
            
            # 发布线加速度（AccelStamped）
            accel_msg = AccelStamped()
            accel_msg.header = imu_transformed.header
            accel_msg.accel.linear = imu_transformed.linear_acceleration  
            self.accel_pub.publish(accel_msg)
            
            # 📌 新增：发布姿态信息（PoseStamped）
            pose_msg = PoseStamped()
            pose_msg.header = imu_transformed.header
            pose_msg.pose.orientation = imu_transformed.orientation
            # 如果有位置信息可以在此附加，但IMU不提供位姿的平移部分，默认设置为原点
            pose_msg.pose.position.x = 0.0  
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 0.0
            self.pose_pub.publish(pose_msg)
            
        except Exception as e:
            rospy.logwarn(f"处理异常: {str(e)}")

if __name__ == '__main__':
    node = ImuTransformer()

