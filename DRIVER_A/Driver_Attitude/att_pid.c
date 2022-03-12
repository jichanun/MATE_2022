#include "attitude control.h"
void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
{
	PID->Error  = target - measure;              //���
	PID->Differ = PID->Error - PID->PreError;    //΢����
	PID->Pout = PID->P * PID->Error;                        //��������
	PID->Iout = PID->Ilimit_flag * PID->I * PID->Integral;  //���ֿ���
	PID->Dout = PID->D * PID->Differ;                       //΢�ֿ���
	PID->OutPut =  PID->Pout + PID->Iout + PID->Dout;       //���� + ���� + ΢���ܿ���
	PID->Integral += PID->Error;                        //�������л���
	if(PID->Integral > PID->Irang)                      //�����޷�
		PID->Integral = PID->Irang;
	if(PID->Integral < -PID->Irang)                     //�����޷�
		PID->Integral = -PID->Irang;                    
	PID->PreError = PID->Error ;                            //ǰһ�����ֵ
}

