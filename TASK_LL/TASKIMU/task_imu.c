#include "task_imu.h"
#include "driver_mpu9250.h"
#include "kalman.h"
#include "bsp_tim.h"
#include "math.h"

#define RAD_TO_DEG 57.295779513082320876798154814105  // ����ת�Ƕ�
#define DEG_TO_RAD 0.01745329251994329576923690768489 // �Ƕ�ת����

#define USE_FIRST_WAY_CALC_PITCH_ROLL 1

typedef struct{
	KALMAN x;
	KALMAN y;
	KALMAN z;
}IMUKalmanStruct;

typedef struct
{
    short Accel_X;  //�Ĵ���ԭ��X����ٶȱ�ʾֵ
    short Accel_Y;  //�Ĵ���ԭ��Y����ٶȱ�ʾֵ
    short Accel_Z;  //�Ĵ���ԭ��Z����ٶȱ�ʾֵ

    short Gyro_X;   //�Ĵ���ԭ��X�������Ǳ�ʾֵ
    short Gyro_Y;   //�Ĵ���ԭ��Y�������Ǳ�ʾֵ
    short Gyro_Z;   //�Ĵ���ԭ��Z�������Ǳ�ʾֵ
	
		short Mag_X;    //�Ĵ���ԭ��X������Ʊ�ʾֵ
		short Mag_Y;    //�Ĵ���ԭ��Y������Ʊ�ʾֵ
		short Mag_Z;    //�Ĵ���ԭ��Z������Ʊ�ʾֵ
	
//	  short Temp;     //�Ĵ���ԭ���¶ȱ�ʾֵ
}MPU9250_RAW_DATA;

typedef struct
{
		double Row;
		double Pitch;
		double Yaw;
		double AccelRow;
	  double AccelPitch;
		double MagYaw;
}MPU9250_STATE;

typedef struct
{
		IMUKalmanStruct     IMUKalman;
		MPU9250_RAW_DATA    MPU9250_Raw_Data;    //ԭʼ����
		MPU9250_STATE       MPU9250_State;       //������
}IMULogic;

IMULogic IMU;

void IMUKalmanInit(void)
{
    KalmanInit(&IMU.IMUKalman.x);
	  KalmanInit(&IMU.IMUKalman.y);
	  KalmanInit(&IMU.IMUKalman.z);
}

void GetMPU9250AccGyroRawData(void)
{
	
		MPU_Get_Accelerometer(&IMU.MPU9250_Raw_Data.Accel_X , &IMU.MPU9250_Raw_Data.Accel_Y , &IMU.MPU9250_Raw_Data.Accel_Z);
		MPU_Get_Gyroscope(&IMU.MPU9250_Raw_Data.Gyro_X , &IMU.MPU9250_Raw_Data.Gyro_Y , &IMU.MPU9250_Raw_Data.Gyro_Z);
}
void GetMPU9250MagRawData(void)
{
		MPU_Get_Magnetometer(&IMU.MPU9250_Raw_Data.Mag_X , &IMU.MPU9250_Raw_Data.Mag_Y , &IMU.MPU9250_Raw_Data.Mag_Z);
}

void AccelUpdatePitchRoll(void) //���ݼ��ٶȼ�ˢ��PITCH,ROLL�����ַ���
{
#if USE_FIRST_WAY_CALC_PITCH_ROLL
    IMU.MPU9250_State.AccelRow   = atan2(IMU.MPU9250_Raw_Data.Accel_Y,IMU.MPU9250_Raw_Data.Accel_Z) * RAD_TO_DEG;
    IMU.MPU9250_State.AccelPitch = atan(-IMU.MPU9250_Raw_Data.Accel_X / sqrt(IMU.MPU9250_Raw_Data.Accel_Y * IMU.MPU9250_Raw_Data.Accel_Y + IMU.MPU9250_Raw_Data.Accel_Z * IMU.MPU9250_Raw_Data.Accel_Y)) * RAD_TO_DEG;
#else
		IMU.MPU9250_State.AccelRow   = atan(IMU.MPU9250_Raw_Data.Accel_Y / sqrt(IMU.MPU9250_Raw_Data.Accel_X * IMU.MPU9250_Raw_Data.Accel_X + IMU.MPU9250_Raw_Data.Accel_Z * IMU.MPU9250_Raw_Data.Accel_Z)) * RAD_TO_DEG;
    IMU.MPU9250_State.AccelPitch = atan2(-IMU.MPU9250_Raw_Data.Accel_X, IMU.MPU9250_Raw_Data.Accel_Z) * RAD_TO_DEG;
#endif
}
 
void MagUpdateYaw() //���ݴ�����ˢ��YAW
{ 
    double RollAngle,PitchAngle,Bfy,Bfx;
		short MagX,MagY,MagZ;
  
			MagX = -IMU.MPU9250_Raw_Data.Mag_X; 
			MagY = IMU.MPU9250_Raw_Data.Mag_Y;
			MagZ = -IMU.MPU9250_Raw_Data.Mag_Z;
   
//    magX *= magGain[0];
//    magY *= magGain[1];
//    magZ *= magGain[2];
//   
//    magX -= magOffset[0];
//    magY -= magOffset[1];
//    magZ -= magOffset[2];   
   
    RollAngle  = IMU.IMUKalman.x.angle * DEG_TO_RAD;
    PitchAngle = IMU.IMUKalman.y.angle * DEG_TO_RAD;
   
    Bfy = MagZ * sin(RollAngle) - MagY * cos(RollAngle);
    Bfx = MagX * cos(PitchAngle) + MagY * sin(PitchAngle) * sin(RollAngle) + MagZ * sin(PitchAngle) * cos(RollAngle);
    IMU.MPU9250_State.MagYaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;
   
//    yaw *= -1;
}

volatile uint32_t LastUpdate, Now;
double dt,dtAdd = 0;
void IMU_CalcYawPitchRollValue(void)
{		
		double GyroXval,GyroYval,GyroZval;
	
		GetMPU9250AccGyroRawData();
		AccelUpdatePitchRoll();
	
		if(dtAdd>0.01)//ÿ10ms��ȡһ�δ�����
		{
				GetMPU9250MagRawData();
				MagUpdateYaw();
				dtAdd = 0;
		}
		
		GyroXval = (double)IMU.MPU9250_Raw_Data.Gyro_X / 32.8;
		GyroYval = (double)IMU.MPU9250_Raw_Data.Gyro_Y / 32.8;
		GyroZval = (double)IMU.MPU9250_Raw_Data.Gyro_Z / 32.8;
	
		Now = GetTimeMicros();//��ȡʱ�� ��λ��us
    if(Now<LastUpdate)
				dt =  (double)((double)(Now + (0xffffffff- LastUpdate)) / 1000000.0f); 
    else
        dt =  (double)((double)(Now - LastUpdate) / 1000000.0f);
    LastUpdate = Now;	//����ʱ��
		dtAdd += dt;

 // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees		
		if((IMU.MPU9250_State.AccelRow<-90&&IMU.IMUKalman.x.angle>90)||(IMU.MPU9250_State.AccelRow>90&&IMU.IMUKalman.x.angle<-90))
				SetAngle(&IMU.IMUKalman.x,IMU.MPU9250_State.AccelRow);
		else	
				KalmanCalc(&IMU.IMUKalman.x , IMU.MPU9250_State.AccelRow , GyroXval ,dt); //Row    ��������ΪԤ�⣬�Լ��ٶȼƼ���ֵΪ����

		if(fabs(IMU.IMUKalman.x.angle)>90)
				GyroYval = -GyroYval;
		KalmanCalc(&IMU.IMUKalman.y ,IMU.MPU9250_State.AccelPitch , GyroYval , dt);//Pitch
		
		if((IMU.MPU9250_State.MagYaw < -90 && IMU.IMUKalman.z.angle > 90) || (IMU.MPU9250_State.MagYaw > 90 && IMU.IMUKalman.z.angle < -90))
				SetAngle(&IMU.IMUKalman.z,IMU.MPU9250_State.MagYaw);
		else
				KalmanCalc(&IMU.IMUKalman.z , IMU.MPU9250_State.MagYaw , GyroZval , dt);//Yaw
				
		IMU.MPU9250_State.Row = IMU.IMUKalman.x.angle;
		IMU.MPU9250_State.Pitch = IMU.IMUKalman.y.angle;
		IMU.MPU9250_State.Yaw = IMU.IMUKalman.z.angle;
}



