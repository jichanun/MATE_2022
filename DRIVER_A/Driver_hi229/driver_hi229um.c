#include "driver_hi229um.h"
#include "math.h"
#include "Variables.h"

#define GYRO_FRAME_LENGTH (40u)



void get_HI229UMGyroData(GYRO1_DataTypeDef* gyro_data)
{   
	uint8_t *GyroData = GYRO1_RAW_Data.DataBuf;
	//解算速度
		gyro_data->AngularVelocity.z_Pitch = (short)((GyroData[17]<<8)+ GyroData[16]) / 10.0f; 
		gyro_data->AngularVelocity.x_Roll = (short)((GyroData[19]<<8)+ GyroData[18]) / 10.0f;
		gyro_data->AngularVelocity.y_Yaw = (short)((GyroData[21]<<8)+ GyroData[20]) / 10.0f;
	//解算角度
		gyro_data->Angle.x_Roll  = (int16_t)((GyroData[31]<<8)+ GyroData[30]) / 100.0f ;
		gyro_data->Angle.z_Pitch    = (int16_t)((GyroData[33]<<8)+ GyroData[32]) / 100.0f;
		gyro_data->Angle.y_Yaw     = (int16_t)((GyroData[35]<<8)+ GyroData[34]) / 10.0f;
	//解算加速度
	
		gyro_data->Acceleration.x_Roll = (int16_t)((GyroData[10]<<8)+ GyroData[9]) / 10.0f; 
		gyro_data->Acceleration.y_Yaw = (int16_t)((GyroData[12]<<8)+ GyroData[11]) / 10.0f;
		gyro_data->Acceleration.z_Pitch = (int16_t)((GyroData[14]<<8)+ GyroData[13]) / 10.0f;
}

//uint8_t HI229_Get_Gyroscope_Location(float *yaw, float *pitch, float *roll)
//{
//	*roll  = (int16_t)((GyroData[31]<<8)+ GyroData[30]) / 100.0f;
//	*pitch = (int16_t)((GyroData[33]<<8)+ GyroData[32]) / 100.0f;
//	*yaw   = (int16_t)((GyroData[35]<<8)+ GyroData[34]) / 10.0f ;
//	return 0;
//}6


//uint8_t HI229_Get_Gyroscope_Speed(float *gx, float *gy, float *gz)
//{
//	*gx = (double)((GyroData[17]<<8)+ GyroData[16]) / 10.0f/360.0f*6.28; 
//	*gy = (double)((GyroData[19]<<8)+ GyroData[18]) / 10.0f/360.0f*6.28;
//	*gz = (double)((GyroData[21]<<8)+ GyroData[20]) / 10.0f/360.0f*6.28;
//	return 0;
//}


//uint8_t HI229_Get_Gyroscope_Acceleration(float *ax, float *ay, float *az)
//{
//	*ax = (int16_t)((GyroData[10]<<8)+ GyroData[9]) / 10.0f; 
//	*ay = (int16_t)((GyroData[12]<<8)+ GyroData[11]) / 10.0f;
//	*az = (int16_t)((GyroData[14]<<8)+ GyroData[13]) / 10.0f;
//	return 0;
//}
float GetYawGyroValue(void)
{
	return	GYRO1_DATA.Angle.y_Yaw;
}
