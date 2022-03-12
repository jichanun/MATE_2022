#include "attitude control.h" 
#include "Driver_Propellor.h"
#include "main.h"
#include "Variables.h"

//�ǶȻ�PID 
PID_TYPE PID_ROL_Angle;
PID_TYPE PID_PIT_Angle;
PID_TYPE PID_YAW_Angle;

//���ٶȻ�PID 
PID_TYPE PID_ROL_Rate;
PID_TYPE PID_PIT_Rate;
PID_TYPE PID_YAW_Rate;


float angle_rol;
float angle_pit;

float Pre_THROTTLE,THROTTLE;
float Moto_PWM_1=0.0f,Moto_PWM_2=0.0f,Moto_PWM_3=0.0f,Moto_PWM_4=0.0f,Moto_PWM_5=0.0f,Moto_PWM_6=0.0f;

extern int32_t Pressure;
extern s16 duoxiao;
uint32_t alt_target;



void Control(FLOAT_ANGLE *att_in,FLOAT_XYZ *gyr_in)
{
	FLOAT_ANGLE Measure_Angle,Target_Angle;
	Measure_Angle.rol = att_in->rol; 
	Measure_Angle.pit = att_in->pit;
	
	Target_Angle.rol = (float)angle_rol;
	Target_Angle.pit = (float)angle_pit;

	

	//�ǶȻ�
	PID_Postion_Cal(&PID_ROL_Angle,Target_Angle.rol,Measure_Angle.rol);//ROLL�ǶȻ�PID ������Ƕ� ������ٶȣ�
	PID_Postion_Cal(&PID_PIT_Angle,Target_Angle.pit,Measure_Angle.pit);//PITH�ǶȻ�PID ������Ƕ� ������ٶȣ�
	
	//���ٶȻ�
	PID_Postion_Cal(&PID_ROL_Rate,PID_ROL_Angle.OutPut,(gyr_in->Y*RadtoDeg)); //ROLL���ٶȻ�PID ������ǶȻ����������������������
	PID_Postion_Cal(&PID_PIT_Rate,PID_PIT_Angle.OutPut,-(gyr_in->X*RadtoDeg)); //PITH���ٶȻ�PID ������ǶȻ����������������������
	

	if(PID_ROL_Rate.OutPut>30)
		PID_ROL_Rate.OutPut=30;
	if(PID_ROL_Rate.OutPut<-30)
		PID_ROL_Rate.OutPut=-30;
	
	if(PID_PIT_Rate.OutPut>30)
		PID_PIT_Rate.OutPut=30;
	if(PID_PIT_Rate.OutPut<-30)
		PID_PIT_Rate.OutPut=-30;
	
	PROP_Speed.VFL =  PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut+100.0f ;  
	PROP_Speed.VFL = -PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut+100.0f ; 
	PROP_Speed.VBL = -PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut+100.0f ;   
	PROP_Speed.VBR =  PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut+100.0f ;   
	
	
}

