#include "task_servo.h"
#include "driver_dataglove.h"
#include "Driver_Servo.h"

/*虚拟开关Dummyswitch的判断*/
/**
	*注意：
	*				左侧拨杆共有三种模式，上、中时都可以用虚拟手套进行控制
	*       下模式时用遥控器来进行控制。不管哪种模式都会继续提供反
	*				馈的速度信息。
	*/
int Dummyswitch =1;

void ModeChooseandExcute(RemoteDataPortStruct	RemoteDataPort)
{
	  Dummyswitch=RemoteDataPort.Grasp;
		if(Dummyswitch==0)//选用数据手套方案  100ms读数间隔
		{
				angle_DG.S_1=DTGL_Data.hand_x*11.11f;
			  angle_DG.S_2=DTGL_Data.hand_y*11.11f;
			  angle_DG.S_3=-1.33f*(DTGL_Data.finger1+DTGL_Data.finger2+DTGL_Data.finger3+DTGL_Data.finger4+DTGL_Data.finger5-250);
				 if((angle_DG.S_1>=-800&&angle_DG.S_1<=800)&&(angle_DG.S_1>=-800&&angle_DG.S_1<=800)&&(angle_DG.S_1>=-800&&angle_DG.S_1<=800))//舵机保护
				SEVO_AngleSet(&angle_DG) ;
		}
		else if(Dummyswitch==1)//遥控器备用方案
		{
				angle_DG.S_1+=RemoteDataPort.Duoji_1*1.5f;
			  angle_DG.S_2+=RemoteDataPort.Duoji_2*1.5f;
			  angle_DG.S_3+=RemoteDataPort.Duoji_3*1.5f;
			  if((angle_DG.S_1>=-800&&angle_DG.S_1<=800)&&(angle_DG.S_2>=-800&&angle_DG.S_2<=800)&&(angle_DG.S_3>=-800&&angle_DG.S_3<=800))//舵机保护
				SEVO_AngleSet(&angle_DG) ;
		}
}
