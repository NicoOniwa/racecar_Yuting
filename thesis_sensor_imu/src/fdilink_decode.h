/*!
 *	\file		fdilink_decode.h
 *  \author		FDISYSTEMS (Edison)
 *	\date		05 February 2023
 *
 *	\brief		Contains main FDILink methods.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2018-2023, FDISYSTEMS  . All rights reserved.
 *	
 *	This source code is intended for use only by FDISYSTEMS and
 *	those that have explicit written permission to use it from
 *	FDISYSTEMS.
 *	
 *	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 *	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *	PARTICULAR PURPOSE.
 */

/*!
 *	\mainpage FDISYSTEMS Communication library documentation
 *	Welcome to the FDILink library documentation.<br>
 *	This documentation describes all functions implemented in the FDILink library.
 */

#ifndef __FDILINK_DECODE_H
#define __FDILINK_DECODE_H
#include "stdint.h"
typedef int64_t timestamp_t;
#pragma pack(1)
typedef struct
{
	uint32_t      SubFrame;      // SubFrame=0
	uint32_t      VariableType;	//变量数据类型
	uint64_t      VariableValue;
	char          VariableName[200];
}FDILINK_COMPONENT_31_0;

typedef struct
{
	uint32_t      SubFrame;      // SubFrame=1
	char          VariableName[200];
}FDILINK_COMPONENT_31_1;

typedef FDILINK_COMPONENT_31_0 FDILink_VarPushData;
typedef FDILINK_COMPONENT_31_1 FDILink_VarPollData;
#define FDILINK_PARAMETER_PACKET_ID 0x31

typedef struct
{
	float Gyroscope_X;
	float Gyroscope_Y;
	float Gyroscope_Z;
	float Accelerometer_X;
	float Accelerometer_Y;
	float Accelerometer_Z;
	float Magnetometer_X;
	float Magnetometer_Y;
	float Magnetometer_Z;
	float IMU_Temperature;
	float Pressure;
	float Pressure_Temperature;
	timestamp_t Timestamp;
}FDILINK_COMPONENT_40;

typedef FDILINK_COMPONENT_40 FDILink_IMUData_Packet_t;
#define FDILINK_IMUDATA_PACKET_ID 0x40

typedef struct
{
	float RollSpeed;
	float PitchSpeed;
	float HeadingSpeed;
	float Roll;
	float Pitch;
	float Heading;
	float Q1;
	float Q2;
	float Q3;
	float Q4;
	timestamp_t Timestamp;
}FDILINK_COMPONENT_41;

typedef FDILINK_COMPONENT_41 FDILink_AHRSData_Packet_t;
#define FDILINK_AHRSDATA_PACKET_ID 0x41

typedef struct
{
	float BodyVelocity_X;
	float BodyVelocity_Y;
	float BodyVelocity_Z;
	float BodyAcceleration_X;
	float BodyAcceleration_Y;
	float BodyAcceleration_Z;
	double Location_North;
	double Location_East;
	double Location_Down;
	float Velocity_North;
	float Velocity_East;
	float Velocity_Down;
	float Acceleration_North;
	float Acceleration_East;
	float Acceleration_Down;
	float Pressure_Altitude;
	timestamp_t Timestamp;
}FDILINK_COMPONENT_42;

typedef FDILINK_COMPONENT_42 FDILink_INSGPSData_Packet_t;
#define FDILINK_INSGPSDATA_PACKET_ID 0x42

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;
typedef float fp32;
typedef double fp64;

typedef union
{
	struct{
		uint16_t System_Failure : 1;
		uint16_t Accelerometer_Sensor_Failure : 1;
		uint16_t Gyroscope_Sensor_Failure : 1;
		uint16_t Magnetometer_Sensor_Failure : 1;
		uint16_t Pressure_Sensor_Failure : 1;
		uint16_t GNSS_Failure : 1;
		uint16_t Accelerometer_Over_Range : 1;
		uint16_t Gyroscope_Over_Range : 1;
		uint16_t Magnetometer_Over_Range : 1;
		uint16_t Pressure_Over_Range : 1;
		uint16_t  Minimum_Temperature_Alarm : 1;
		uint16_t  Maximum_Temperature_Alarm : 1;
		uint16_t  Low_Voltage_Alarm : 1;
		uint16_t  High_Voltage_Alarm : 1;
		uint16_t  GNSS_Antenna_Disconnected : 1;
		uint16_t  Data_Output_OverBow_Alarm : 1;
	};
	uint16_t w;
}System_status_t;
typedef struct
{
	struct{
		uint16_t  Orientation_Filter_Initialised : 1;
		uint16_t  Navigation_Filter_Initialised : 1;
		uint16_t  Heading_Initialised : 1;
		uint16_t  UTC_Time_Initialised : 1;
		uint16_t  GNSS_Fix_Status : 4;
		uint16_t  Event_Occurred : 1;
		uint16_t  Internal_GNSS_Enabled : 1;
		uint16_t  Magnetic_Heading_Active : 1;
		uint16_t  Velocity_Heading_Enabled : 1;
		uint16_t  Atmospheric_Altitude_Enabled : 1;
		uint16_t  External_Position_Active : 1;
		uint16_t  External_Velocity_Active : 1;
		uint16_t  External_Heading_Active : 1;
	};
	uint16_t w;
}Filter_Status_t;

typedef struct
{
	u16 System_status;// see section 13.9.1.1
	u16 Filter_status;// see section 13.9.1.2
	u32 Unix_time;// (seconds) see section 13.9.1.4
	u32 Microseconds;// see section 13.9.1.5
	fp64 Latitude;// (rad)
	fp64 Longitude;// (rad)
	fp64 Height;// (m)
	fp32 Velocity_north;// (m/s)
	fp32 Velocity_east;// (m/s)
	fp32 Velocity_down;// (m/s)
	fp32 Body_acceleration_X;// (m/s/s)
	fp32 Body_acceleration_Y;// (m/s/s)
	fp32 Body_acceleration_Z;// (m/s/s)
	fp32 G_force;// (g)
	fp32 Roll;// (radians)
	fp32 Pitch;// (radians)
	fp32 Heading;// (radians)
	fp32 Angular_velocity_X;// (rad/s)
	fp32 Angular_velocity_Y;// (rad/s)
	fp32 Angular_velocity_Z;// (rad/s)
	fp32 Latitude_standard_deviation;// (m)
	fp32 Longitude_standard_deviation;// (m)
	fp32 Height_standard_deviation;// (m)
}System_State_Packet_t;
#define System_State_Packet_ID 0x50

typedef struct
{
	u32 Unix_time;
	u32 Microseconds;
}Unix_Time_Packet_t;
#define Unix_Time_Packet_ID 0x51

typedef struct
{
	u32   Microseconds;
	u16   Year;
	u16   Year_day;
	u8   Month;
	u8   Month_Day;
	u8   Week_Day;
	u8   Hour;
	u8   Minute;
	u8   Second;
}Formatted_Time_Packet_t;
#define Formatted_Time_Packet_ID 0x52

typedef struct
{
	u16   System_status;
	u16   Filter_status;

}Status_Packet_t;
#define Status_Packet_ID 0x53

typedef struct
{
	fp32   Latitude_standard_deviation;
	fp32   Longitude_standard_deviation;
	fp32   Height_standard_deviation;
}Position_Standard_Deviation_Packet_t;
#define Position_Standard_Deviation_Packet_ID 0x54

typedef struct
{
	fp32   Velocity_north_standard_deviation;
	fp32   Velocity_east_standard_deviation;
	fp32   Velocity_down_standard_deviation;
}Velocity_Standard_Deviation_Packet_t;
#define Velocity_Standard_Deviation_Packet_ID 0x55

typedef struct
{
	fp32   Roll_standard_deviation;
	fp32   Pitch_standard_deviation;
	fp32   Heading_standard_deviation;
}Euler_Orientation_Standard_Deviation_Packet_t;
#define Euler_Orientation_Standard_Deviation_Packet_ID 0x56

typedef struct
{
	fp32   Q0_standard_deviation;
	fp32   Q1_standard_deviation;
	fp32   Q2_standard_deviation;
	fp32   Q3_standard_deviation;

}Quaternion_Orientation_Standard_Deviation_Packet_t;
#define Quaternion_Orientation_Standard_Deviation_Packet_ID 0x57

typedef struct
{
	fp32   Accelerometer_X;
	fp32   Accelerometer_Y;
	fp32   Accelerometer_Z;
	fp32   Gyroscope_X;
	fp32   Gyroscope_Y;
	fp32   Gyroscope_Z;
	fp32   Magnetometer_X;
	fp32   Magnetometer_Y;
	fp32   Magnetometer_Z;
	fp32   IMU_Temperature;
	fp32   Pressure;
	fp32   Pressure_Temperature;

}Raw_Sensors_Packet_t;
#define Raw_Sensors_Packet_ID 0x58

typedef struct
{
	u32   Unix_time_stamp;
	u32   Microseconds;
	fp64   Latitude;
	fp64   Longitude;
	fp64   Height;
	fp32   Velocity_north;
	fp32   Velocity_east;
	fp32   Velocity_down;
	fp32   Latitude_standard_deviation;
	fp32   Longitude_standard_deviation;
	fp32   Height_standard_deviation;
	fp32   Reserved1;
	fp32   Reserved2;
	fp32   Reserved3;
	fp32   Reserved4;
	u16   Status;

}Raw_GNSS_Packet_t;
#define Raw_GNSS_Packet_ID 0x59

typedef struct
{
	fp32   HDOP;
	fp32   VDOP;
	u8   GPS_satellites;
	u8   GLONASS_satellites;
	u8   BeiDou_satellites;
	u8   GALILEO_satellites;
	u8   SBAS_satellites;
}Satellites_Packet_t;
#define Satellites_Packet_ID 0x5A

typedef struct
{
	u8   Satellite_system;
	u8   Satellite_number;
	s8   Satellite_frequencies;
	u8   Elevation;
	u16   Azimuth;
	u8   SNR;
}Detailed_Satellites_Packet_t;
#define Detailed_Satellites_Packet_ID 0x5B

typedef struct
{
	fp64   Latitude;
	fp64   Longitude;
	fp64   Height;
}Geodetic_Position_Packet_t;
#define Geodetic_Position_Packet_ID 0x5C

typedef struct
{
	fp64   ECEF_X;
	fp64   ECEF_Y;
	fp64   ECEF_Z;
}ECEF_Position_Packet_t;
#define ECEF_Position_Packet_ID 0x5D

typedef struct
{
	fp64   Northing;
	fp64   Easting;
	fp64   Height;
	u8   Zone_number;
	s8   Zone_character;
}UTM_Position_Packet_t;
#define UTM_Position_Packet_ID 0x5E

typedef struct
{
	fp32   Velocity_north;
	fp32   Velocity_east;
	fp32   Velocity_down;
}NED_Velocity_Packet_t;
#define NED_Velocity_Packet_ID 0x5F

typedef struct
{
	fp32   Velocity_X;
	fp32   Velocity_Y;
	fp32   Velocity_Z;
}Body_Velocity_Packet_t;
#define Body_Velocity_Packet_ID 0x60

typedef struct
{
	fp32   Acceleration_X;
	fp32   Acceleration_Y;
	fp32   Acceleration_Z;
}Acceleration_Packet_t;
#define Acceleration_Packet_ID 0x61

typedef struct
{
	fp32   Body_acceleration_X;
	fp32   Body_acceleration_Y;
	fp32   Body_acceleration_Z;
	fp32   G_force;
}Body_Acceleration_Packet_t;
#define Body_Acceleration_Packet_ID 0x62

typedef struct
{
	fp32   Roll;
	fp32   Pitch;
	fp32   Heading;
}Euler_Orientation_Packet_t;
#define Euler_Orientation_Packet_ID 0x63

typedef struct
{
	fp32   Q0;
	fp32   Q1;
	fp32   Q2;
	fp32   Q3;
}Quaternion_Orientation_Packet_t;
#define Quaternion_Orientation_Packet_ID 0x64

typedef struct
{
	fp32 DCM[3][3];
}DCM_Orientation_Packet_t;
#define DCM_Orientation_Packet_ID 0x65

typedef struct
{
	fp32   Angular_velocity_X;
	fp32   Angular_velocity_Y;
	fp32   Angular_velocity_Z;
}Angular_Velocity_Packet_t;
#define Angular_Velocity_Packet_ID 0x66

typedef struct
{
	fp32   Angular_acceleration_X;
	fp32   Angular_acceleration_Y;
	fp32   Angular_acceleration_Z;
}Angular_Acceleration_Packet_t;
#define Angular_Acceleration_Packet_ID 0x67

typedef struct
{
	fp64   Latitude;
	fp64   Longitude;
	fp64   Height;
	fp32   Velocity_north;
	fp32   Velocity_east;
	fp32   Velocity_down;
	fp32   Latitude_standard_deviation;
	fp32   Longitude_standard_deviation;
	fp32   Height_standard_deviation;
	fp32   Velocity_north_standard_deviation;
	fp32   Velocity_east_standard_deviation;
	fp32   Velocity_down_standard_deviation;
}External_Position_And_Velocity_Packet_t;
#define External_Position_And_Velocity_Packet_ID 0x68

typedef struct
{
	fp64   Latitude;
	fp64   Longitude;
	fp64   Height;
	fp32   Latitude_standard_deviation;
	fp32   Longitude_standard_deviation;
	fp32   Height_standard_deviation;
}External_Position_Packet_t;
#define External_Position_Packet_ID 0x69

typedef struct
{
	fp32   Velocity_north;
	fp32   Velocity_east;
	fp32   Velocity_down;
	fp32   Velocity_north_standard_deviation;
	fp32   Velocity_east_standard_deviation;
	fp32   Velocity_down_standard_deviation;
}External_Velocity_Packet_t;
#define External_Velocity_Packet_ID 0x6A

typedef struct
{
	fp32   Velocity_X;
	fp32   Velocity_Y;
	fp32   Velocity_Z;
	fp32   Velocity_standard_deviation;
}External_Body_Velocity_Packet_t;
#define External_Body_Velocity_Packet_ID 0x6B

typedef struct
{
	fp32   Heading;
	fp32   Heading_standard_deviation;
}External_Heading_Packet_t;
#define External_Heading_Packet_ID 0x6C

typedef struct
{
	u32   Running_time_seconds;
	u32   Microseconds;
}Running_Time_Packet_t;
#define Running_Time_Packet_ID 0x6D

typedef struct
{
	fp32   Local_magnetic_field_X;
	fp32   Local_magnetic_field_Y;
	fp32   Local_magnetic_field_Z;
}Local_Magnetic_Field_Packet_t;
#define Local_Magnetic_Field_Packet_ID 0x6E

typedef struct
{
	s32   Odometer_pulse_count;
	fp32   Odometer_distance;
	fp32   Odometer_speed;
	fp32   Odometer_slip;
	u8   Odometer_active;
	u8  Reserved[3];
}Odometer_State_Packet_t;
#define Odometer_State_Packet_ID 0x6F

typedef struct
{
	u32   Unix_time_seconds;
	u32   Microseconds;
}External_Time_Packet_t;
#define External_Time_Packet_ID 0x70

typedef struct
{
	fp32   Depth;
	fp32   Depth_standard_deviation;
}External_Depth_Packet_t;
#define External_Depth_Packet_ID 0x71

typedef struct
{
	fp32   Geoid_height;
}Geoid_Height_Packet_t;
#define Geoid_Height_Packet_ID 0x72

typedef struct
{
	u8 RTCM_corrections_data[255];
}RTCM_Corrections_Packet_t;
#define RTCM_Corrections_Packet_ID 0x73

typedef struct
{
	fp32   Differential_pressure;
	fp32   Outside_air_temperature;
}External_Pitot_Pressure_Packet_t;
#define External_Pitot_Pressure_Packet_ID 0x74

typedef struct
{
	fp32   Wind_velocity_north;
	fp32   Wind_velocity_east;
	fp32   Wind_velocity_standard_deviation;
}Wind_Packet_t;
#define Wind_Packet_ID 0x75

typedef struct
{
	fp32   Heave_point_1;
	fp32   Heave_point_2;
	fp32   Heave_point_3;
	fp32   Heave_point_4;
}Heave_Packet_t;
#define Heave_Packet_ID 0x76

typedef struct
{
	u32   Unix_time;
	u32   Nanoseconds;
	s32   Receiver_clock_offset;
	u8   Receiver_number;
	u8   Packet_number;
	u8   Total_packets;
	u8   Number_of_satellites;
	u8   Satellite_system;
	u8   PRN_or_satellite_number;
	u8   Elevation;
	u16   Azimuth;
	u8   Number_of_frequencies;
	u8   Satellite_frequency;
	u8   Tracking_status;
	fp64   Carrier_phase;
	fp64   Pseudo_range;
	fp32   Doppler_frequency;
	fp32   Signal_to_noise_ratio;
}Raw_Satellite_Data_Packet_t;
#define Raw_Satellite_Data_Packet_ID 0x77

typedef struct
{
	fp32   Estimated_delay;
	fp32   Speed;
	fp32   Reserved;
	u8   Odometer_Bags;
}External_Odometer_Packet_t;
#define External_Odometer_Packet_ID 0x78

typedef struct
{
	fp32   Barometric_altitude_delay;
	fp32   Airspeed_delay;
	fp32   Barometric_altitude;
	fp32   Airspeed;
	fp32   Barometric_altitude_standard_deviation;
	fp32   Airspeed_standard_deviation;
	u8   Flags;
}External_Air_Data_Packet_t;
#define External_Air_Data_Packet_ID 0x79

typedef struct
{
	fp32   Current_angle;
	u32   Reserved;
}Gimbal_State_Packet_t;
#define Gimbal_State_Packet_ID 0x7A

typedef struct
{
	fp32   Virtual_odometer_distance;
	fp32   Slip_Angle;
	fp32   Velocity_X;
	fp32   Velocity_Y;
	fp32   Distance_standard_deviation;
	u32 Reserved;
}Automotive_Packet_t;
#define Automotive_Packet_ID 0x7B

typedef struct
{
	u8   Permanent;
	u8   UTC_synchronisation;
	u16   Packet_timer_period;
}Packet_Timer_Period_Packet_t;
#define Packet_Timer_Period_Packet_ID 0x7C

typedef struct
{
	u8   Packet_ID;
	u32   Packet_period;
}Packets_Period_Packet_Sub_t;

typedef struct
{
	u8   Permanent;
	u8   Clear_existing_packet_periods;
	Packets_Period_Packet_Sub_t periods[5];
}Packets_Period_Packet_t;
#define Packets_Period_Packet_ID 0x7D


typedef struct
{
	u8   Permanent;
	fp32   Alignment_DCM[3][3];
	fp32   GNSS_antenna_offset_X;
	fp32   GNSS_antenna_offset_Y;
	fp32   GNSS_antenna_offset_Z;
	fp32   Odometer_offset_X;
	fp32   Odometer_offset_Y;
	fp32   Odometer_offset_Z;
	fp32   External_data_offset_X;
	fp32   External_data_offset_Y;
	fp32   External_data_offset_Z;
}Installation_Alignment_Packet_t;
#define Installation_Alignment_Packet_ID 0x80

typedef struct
{
	u8   Permanent;
	u8   Vehicle_type;
	u8   Internal_GNSS_enabled;
	u8   Magnetometers_enabled;
	u8   Atmospheric_altitude_enabled;
	u8   Velocity_heading_enabled;
	u8   Reversing_detection_enabled;
	u8   Motion_analysis_enabled;
	u8   Automatic_magnetic_calibration_enabled;
}Filter_Options_Packet_t;
#define Filter_Options_Packet_ID 0x81

typedef struct
{
	u8   Permanent;
	u8   GPIO1_Function;
	u8   GPIO2_Function;
	u8   Auxiliary_RS232_transmit_function;
	u8   Auxiliary_RS232_receive_function;
}GPIO_Configuration_Packet_t;
#define GPIO_Configuration_Packet_ID 0x82

typedef struct
{
	u8   Permanent;
	fp32   Hard_iron_bias_X;
	fp32   Hard_iron_bias_Y;
	fp32   Hard_iron_bias_Z;
	fp32   Soft_iron_transformation_XX;
	fp32   Soft_iron_transformation_XY;
	fp32   Soft_iron_transformation_XZ;
	fp32   Soft_iron_transformation_YX;
	fp32   Soft_iron_transformation_YY;
	fp32   Soft_iron_transformation_YZ;
	fp32   Soft_iron_transformation_ZX;
	fp32   Soft_iron_transformation_ZY;
	fp32   Soft_iron_transformation_ZZ;
}Magnetic_Calibration_Values_Packet_t;
#define Magnetic_Calibration_Values_Packet_ID 0x83

typedef struct
{
	u8   Magnetic_calibration_action;
}Magnetic_Calibration_Configuration_Packet_t;
#define Magnetic_Calibration_Configuration_Packet_ID 0x84

typedef struct
{
	u8   Magnetic_calibration_status;
	u8   Magnetic_calibration_progress;
	u8   Local_magnetic_error;
}Magnetic_Calibration_Status_Packet_t;
#define Magnetic_Calibration_Status_Packet_ID 0x85

typedef struct
{
	u8   Permanent;
	u8   Automatic_pulse_measurement_active;
	u16   Reserved;
	fp32   Pulse_length;
}Odometer_Configuration_Packet_t;
#define Odometer_Configuration_Packet_ID 0x86

typedef struct
{
	u8   Permanent;
	u32   Verification_sequence;
}Set_Zero_Orientation_Alignment_Packet_t;
#define Set_Zero_Orientation_Alignment_Packet_ID 0x87

typedef struct
{
	u8   Permanent;
	fp32   Primary_reference_point_offset_X;
	fp32   Primary_reference_point_offset_Y;
	fp32   Primary_reference_point_offset_Z;
	fp32   Heave_point_2_offset_X;
	fp32   Heave_point_2_offset_Y;
	fp32   Heave_point_2_offset_Z;
	fp32   Heave_point_3_offset_X;
	fp32   Heave_point_3_offset_Y;
	fp32   Heave_point_3_offset_Z;
	fp32   Heave_point_4_offset_X;
	fp32   Heave_point_4_offset_Y;
	fp32   Heave_point_4_offset_Z;
}Reference_Point_Offsets_Packet_t;
#define Reference_Point_Offsets_Packet_t 0x88

//typedef struct
//{
//	u8   Permanent;
//	u8   NMEA_8x_behaviour;
//	u16   GPZDA_Rates;
//	u16   GPGGA_Rates;
//	u16   GPVTG_Rates;
//	u16   GPRMC_Rates;
//	u16   GPHDT_Rates;
//	u16   GPGLL_Rates;
//	u16   PASHR_Rates;
//	u16   TSS1_Rates;
//	u16   Simrad_Rates;
//	u8   Reserved[12];
//}GPIO_Output_Configuration_Packet_t;
//#define GPIO_Output_Configuration_Packet_ID 0x89

typedef struct
{
	float User_data[16];
}User_Data_Packet_t;
#define User_Data_Packet_ID 0x8A



#endif //__FDILINK_DECODE_H
