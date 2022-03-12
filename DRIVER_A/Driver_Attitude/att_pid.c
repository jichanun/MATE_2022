#include "attitude control.h"
void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
{
	PID->Error  = target - measure;              //误差
	PID->Differ = PID->Error - PID->PreError;    //微分量
	PID->Pout = PID->P * PID->Error;                        //比例控制
	PID->Iout = PID->Ilimit_flag * PID->I * PID->Integral;  //积分控制
	PID->Dout = PID->D * PID->Differ;                       //微分控制
	PID->OutPut =  PID->Pout + PID->Iout + PID->Dout;       //比例 + 积分 + 微分总控制
	PID->Integral += PID->Error;                        //对误差进行积分
	if(PID->Integral > PID->Irang)                      //积分限幅
		PID->Integral = PID->Irang;
	if(PID->Integral < -PID->Irang)                     //积分限幅
		PID->Integral = -PID->Irang;                    
	PID->PreError = PID->Error ;                            //前一个误差值
}

