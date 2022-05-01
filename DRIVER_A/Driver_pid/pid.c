/****************PID运算****************/

#include "pid.h"

//初始化pid参数
void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->maxIntegral=maxI;
	pid->maxOutput=maxOut;
}

//单级pid计算
void PID_SingleCalc(PID *pid,float reference,float feedback)
{
	//更新数据
	pid->lastError=pid->error;
	pid->error=reference-feedback;
//	if(pid->error<1)
//		pid->error = 0;
	//计算微分
	pid->output=(pid->error-pid->lastError)*pid->kd;
	//计算比例
	pid->output+=pid->error*pid->kp;
	//计算积分
	pid->integral+=pid->error*pid->ki;
	LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);//积分限幅
	pid->output+=pid->integral;
	//输出限幅
	LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}

//串级pid计算
void PID_CascadeCalc(CascadePID *pid,float depthRef,float depthFdb,float speedFdb)
{
	PID_SingleCalc(&pid->outer,depthRef,depthFdb);//计算外环(深度环)
	PID_SingleCalc(&pid->inner,pid->outer.output,speedFdb);//计算内环(速度环)
	pid->output=pid->inner.output;
}

//清空一个pid的历史数据
void PID_Clear(PID *pid)
{
	pid->error=0;
	pid->lastError=0;
	pid->integral=0;
	pid->output=0;
}

//重新设定pid输出限幅
void PID_SetMaxOutput(PID *pid,float maxOut)
{
	PID_Clear(pid);
	pid->maxOutput=maxOut;
}

//增量式Pid计算
void SPID_Calc(PID *pid,float reference,float feedback)
{
	//更新数据
	pid->PreError=pid->lastError;
	pid->lastError=pid->error;
	pid->error=reference-feedback;
//	if(pid->error<0.05)
//		pid->error = 0;
	//计算
	pid->output=pid->error*pid->kp-pid->lastError*pid->ki+pid->PreError*pid->kd;
	//输出限幅
	LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}
