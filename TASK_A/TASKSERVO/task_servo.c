#include "task_servo.h"
#include "Driver_Servo.h"

/*虚拟开关Dummyswitch的判断*/
void ModeChooseandExcute(void)
{
		if(Dummyswitch)//选用数据手套方案
		{
				ReadData_Dataglove();
				SEVO_AngleSet(&angle_DG) ;
		}
		else//选用备用方案
		{
				/*读取方式待定*/
				SEVO_AngleSet(&angle_) ;
		}
}