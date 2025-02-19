#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include "fdilink_decode.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <iomanip>
#include <cstdio>
#include "thesis_sensor_imu/Ins_BodyLinear.h"

extern "C" int InuptChar(uint8_t c);

extern FDILink_IMUData_Packet_t IMU_Recive;
extern FDILink_AHRSData_Packet_t AHRS_Recive;
extern FDILink_INSGPSData_Packet_t INSGPS_Recive;

ros::Publisher pub_imu; //imu msg publisher
ros::Publisher pub_mag; //magnetometer msg publisher
ros::Publisher pub_ins_bodylinear; // INS/GPS msg and the body velocity of the INS/GPS msg publisher

int main(int argc,char** argv)
{
    std::cout << "========================"  << std::endl
              << "Ros wrapper for FDI Sensor"<< std::endl
              << "it contains a accelerameter, gyroscope and magnetometer" << std::endl
              << "========================" << std::endl;

    ros::init(argc, argv, "launch_imu_node");
    ros::NodeHandle nh("~"); 

    //read serial port param
    std::string IMU_SERIAL_PORT("/dev/ttyUSB0");
    // these are variable definition, the capital name is the variable name 
    // and the string is their content
    std::string FDI_IMU_TOPIC("/fdi_imu");
    std::string FDI_MAG_TOPIC("/fdi_mag");
    std::string FDI_INS_BODYLINEAR_TOPIC("/fdi_ins_bodylinear");

    /* //Get Param的三种方法
    //① ros::param::get()获取参数“param1”的value，写入到param1上
    bool test1 = ros::param::get("param1", param1);
    
    //② ros::NodeHandle::getParam()获取参数，与①作用相同
    bool test2 = nh.getParam("param2",param2);
    
    //③ ros::NodeHandle::param()类似于①和②
    //但如果get不到指定的param，它可以给param指定一个默认值(如1)
    nh.param("param3", param3, 1); */

    nh.param("FDI_IMU_TOPIC",FDI_IMU_TOPIC);
    nh.param("FDI_MAG_TOPIC",FDI_MAG_TOPIC);
    nh.param("FDI_INS_BODYLINEAR_TOPIC",FDI_INS_BODYLINEAR_TOPIC);

    
    if(!nh.getParam("imu_serial_port",IMU_SERIAL_PORT))
    {
        ROS_WARN_STREAM("The imu serial port not set, use default: /dev/ttyUSB0");
    }

    pub_imu = nh.advertise<sensor_msgs::Imu>(FDI_IMU_TOPIC,1000);
    pub_mag = nh.advertise<sensor_msgs::MagneticField>(FDI_MAG_TOPIC,1000);
    pub_ins_bodylinear = nh.advertise<thesis_sensor_imu::Ins_BodyLinear>(FDI_INS_BODYLINEAR_TOPIC, 1000);
    
    serial::Serial sp; //声明串口对象
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort(IMU_SERIAL_PORT); //此处需要根据imu的接的串口来进行修改
    //sp.setBaudrate(921600);//波特率
    sp.setBaudrate(115200);//波特率
    sp.setTimeout(to);
    try
    {
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port " << IMU_SERIAL_PORT << ", may be a wrong port number or permission denied");
        return -1;
    }
    if(sp.isOpen()) //检测串口是否已经打开，并给出提示
    {
        ROS_INFO_STREAM( 
            IMU_SERIAL_PORT << " is opened. ");
        
    }
    else
    {
        return -1;
    }
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        size_t n = sp.available();
        int curr_ID = 0;
        //std::cout  << "available data is " << n << std::endl;  
        if(n!=0)  // two frame received
        {
            uint8_t buffer[256];
            n = n > 256 ? 256 : n;
            n = sp.read(buffer, n);
            for(int i=0;i<n;i++)
            {
                curr_ID = InuptChar(buffer[i]);
                if(curr_ID == FDILINK_IMUDATA_PACKET_ID)
                {
                    // std::cout 
                    //   << "[" <<IMU_Recive.Accelerometer_X 
                    //   << "," <<IMU_Recive.Accelerometer_Y 
                    //   << "," <<IMU_Recive.Accelerometer_Z 
                    //   << "," <<IMU_Recive.Gyroscope_X
                    //   << "," <<IMU_Recive.Gyroscope_Y
                    //   << "," <<IMU_Recive.Gyroscope_Z
                    //   << "," <<IMU_Recive.Magnetometer_X
                    //   << "," <<IMU_Recive.Magnetometer_Y
                    //   << "," <<IMU_Recive.Magnetometer_Z
                    //   << "]" << std::endl;
                    //assemble ros msgs 
                    sensor_msgs::Imu imu_msg;
                    sensor_msgs::MagneticField mag_msg;
                    ros::Time time = ros::Time::now();
                    //double secs = time.toSec();
                    //std::cout << std::setprecision(20) << "shijian" << secs << std::endl;
                    imu_msg.header.stamp = time;
                    imu_msg.header.frame_id = "fdi_imu";
                    imu_msg.angular_velocity.x = IMU_Recive.Gyroscope_X;
                    imu_msg.angular_velocity.y = IMU_Recive.Gyroscope_Y;
                    imu_msg.angular_velocity.z = IMU_Recive.Gyroscope_Z;
                    imu_msg.linear_acceleration.x = IMU_Recive.Accelerometer_X;
                    imu_msg.linear_acceleration.y = IMU_Recive.Accelerometer_Y;
                    imu_msg.linear_acceleration.z = IMU_Recive.Accelerometer_Z;
                    
                    mag_msg.header = imu_msg.header;
                    mag_msg.magnetic_field.x = IMU_Recive.Magnetometer_X;
                    mag_msg.magnetic_field.y = IMU_Recive.Magnetometer_Y;
                    mag_msg.magnetic_field.z = IMU_Recive.Magnetometer_Z;

                    pub_imu.publish(imu_msg);
                    pub_mag.publish(mag_msg);
                    
                }
                else if(curr_ID == FDILINK_AHRSDATA_PACKET_ID)
                {
                    // Process and publish AHRS data
                    sensor_msgs::Imu imu_msg;
                    ros::Time time = ros::Time::now();
                    imu_msg.header.stamp = time;
                    imu_msg.header.frame_id = "fdi_ahrs";
                    imu_msg.angular_velocity.x = AHRS_Recive.RollSpeed;
                    imu_msg.angular_velocity.y = AHRS_Recive.PitchSpeed;
                    imu_msg.angular_velocity.z = AHRS_Recive.HeadingSpeed;
                    imu_msg.orientation.x = AHRS_Recive.Q1;
                    imu_msg.orientation.y = AHRS_Recive.Q2;
                    imu_msg.orientation.z = AHRS_Recive.Q3;
                    imu_msg.orientation.w = AHRS_Recive.Q4;

                    pub_imu.publish(imu_msg);
                }
                else if(curr_ID == FDILINK_INSGPSDATA_PACKET_ID)
                {
                    // Process and publish INS/GPS data
                    thesis_sensor_imu::Ins_BodyLinear ins_bodylinear_msg;
                    ros::Time time = ros::Time::now();
                    ins_bodylinear_msg.ins_data.header.stamp = time;
                    ins_bodylinear_msg.ins_data.header.frame_id = "fdi_ins_bodylinear";
                    ins_bodylinear_msg.ins_data.latitude = INSGPS_Recive.Location_North;
                    ins_bodylinear_msg.ins_data.longitude = INSGPS_Recive.Location_East;
                    ins_bodylinear_msg.ins_data.altitude = INSGPS_Recive.Location_Down;

                    ins_bodylinear_msg.body_linear.x = INSGPS_Recive.BodyVelocity_X;
                    ins_bodylinear_msg.body_linear.y = INSGPS_Recive.BodyVelocity_Y;
                    ins_bodylinear_msg.body_linear.z = INSGPS_Recive.BodyVelocity_Z;

                    // TODO user-defined message that contain the location and the body velocity
                    pub_ins_bodylinear.publish(ins_bodylinear_msg);
                }
            }
        }
        //std::cout << "[" ;
        loop_rate.sleep();
    }

    return 0;
}