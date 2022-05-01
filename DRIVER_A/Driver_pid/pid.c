/****************PID����****************/

#include "pid.h"

//��ʼ��pid����
void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->maxIntegral=maxI;
	pid->maxOutput=maxOut;
}

//����pid����
void PID_SingleCalc(PID *pid,float reference,float feedback)
{
	//��������
	pid->lastError=pid->error;
	pid->error=reference-feedback;
//	if(pid->error<1)
//		pid->error = 0;
	//����΢��
	pid->output=(pid->error-pid->lastError)*pid->kd;
	//�������
	pid->output+=pid->error*pid->kp;
	//�������
	pid->integral+=pid->error*pid->ki;
	LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);//�����޷�
	pid->output+=pid->integral;
	//����޷�
	LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}

//����pid����
void PID_CascadeCalc(CascadePID *pid,float depthRef,float depthFdb,float speedFdb)
{
	PID_SingleCalc(&pid->outer,depthRef,depthFdb);//�����⻷(��Ȼ�)
	PID_SingleCalc(&pid->inner,pid->outer.output,speedFdb);//�����ڻ�(�ٶȻ�)
	pid->output=pid->inner.output;
}

//���һ��pid����ʷ����
void PID_Clear(PID *pid)
{
	pid->error=0;
	pid->lastError=0;
	pid->integral=0;
	pid->output=0;
}

//�����趨pid����޷�
void PID_SetMaxOutput(PID *pid,float maxOut)
{
	PID_Clear(pid);
	pid->maxOutput=maxOut;
}

//����ʽPid����
void SPID_Calc(PID *pid,float reference,float feedback)
{
	//��������
	pid->PreError=pid->lastError;
	pid->lastError=pid->error;
	pid->error=reference-feedback;
//	if(pid->error<0.05)
//		pid->error = 0;
	//����
	pid->output=pid->error*pid->kp-pid->lastError*pid->ki+pid->PreError*pid->kd;
	//����޷�
	LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}
