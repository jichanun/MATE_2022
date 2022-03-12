#include "stdint.h"
#define RadtoDeg    57.324841f				//????? (?? * 180/3.1415)

//PID??????? 
typedef struct PID
{
  float P;         //??
  float I;
  float D;
  float Error;     //???
  float Integral;  //???
  float Differ;    //???
  float PreError;
  float PrePreError;
  float Ilimit; 
  float Irang;
  float Pout;
  float Iout;
  float Dout;
  float OutPut;   
  uint8_t Ilimit_flag;    //????	
}PID_TYPE;   

typedef int16_t s16;

//????????
typedef struct
{
	float rol;
	float pit;
	float yaw;
}FLOAT_ANGLE;

typedef struct
{
	float X;
	float Y;
	float Z;
}FLOAT_XYZ;

void PID_Postion_Cal(PID_TYPE*PID,float target,float measure);