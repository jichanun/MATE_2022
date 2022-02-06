#include "task_servo.h"
#include "driver_dataglove.h"
#include "Driver_Servo.h"

/*虚拟开关Dummyswitch的判断*/
void ModeChooseandExcute(void)
{
		if(Dummyswitch)//选用数据手套方案  500ms读数间隔
		{
				angle_DG.S_1=Angle_x2*11.11;
			  angle_DG.S_1=Angle_y1*11.11;
			  angle_DG.S_1=-1.33*(finger1+finger2+finger3+finger4+finger5-250);
				SEVO_AngleSet(&angle_DG) ;
		}
		else//选用备用方案
		{
				/*读取方式待定*/
				SEVO_AngleSet(&angle_) ;
		}
}

