#include "task_servo.h"
#include "driver_dataglove.h"
#include "Driver_Servo.h"

/*虚拟开关Dummyswitch的判断*/
/**
	*注意：这里应当采用将DTGL_Data结构体指针作为函数参数的方法来做，
	*				但由于这个函数尚未定型，所以在本.c文件的.h文件中包含了Variables.h，
	*				并且将DTGL_Data直接作为变量放在了函数中。
	*				应当指出的是，这种用法破坏了“分层封装”的设计理念，不应当作为最终版本使用。
	*				后期函数定型后，应当修改这种方式。
	*/
void ModeChooseandExcute(void)
{
		if(Dummyswitch)//选用数据手套方案  500ms读数间隔
		{
				angle_DG.S_1=DTGL_Data.Angle_x2*11.11f;
			  angle_DG.S_2=DTGL_Data.Angle_y1*11.11f;
			  angle_DG.S_3=-1.33f*(DTGL_Data.finger1+DTGL_Data.finger2+DTGL_Data.finger3+DTGL_Data.finger4+DTGL_Data.finger5-250);
				SEVO_AngleSet(&angle_DG) ;
		}
		else//选用备用方案
		{
				/*读取方式待定*/
				SEVO_AngleSet(&angle_) ;
		}
}
