/**
  ******************************************************************************
  * @file           	Driver_Gyroscope.c
  * @brief          	本文件定义维特智能 WITMOTION 公司 JY901S 陀螺仪模块驱动.
	*										实现模块数据解算，配置修改.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.1首次发布
  *
  *	@Contributor			DogeYellow
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Driver_Gyroscope.h"

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private constants ---------------------------------------------------------*/
	/* 解锁指令：发送其他指令前需要发送该指令，等待10ms后执行其他指令 */
uint8_t GYRO_INSRUCT_UNLOCK[5]={0xff, 0xaa, 0x69, 0x88, 0xb5} ;
	/* 保存指令：将配置写入FLASH，需要等待100ms后执行其他指令 */
uint8_t GYRO_INSRUCT_SAVE[5]={0xff, 0xaa, 0x00, 0x00, 0x00} ;
	/* 安装方式配置指令 */
uint8_t GYRO_INSRUCT_VerticalSet[5]={0xff, 0xaa, 0x23, 0x01, 0x00} ;
uint8_t GYRO_INSRUCT_HorizontalSet[5]={0xff, 0xaa, 0x23, 0x00, 0x00} ;
	/* 睡眠、唤醒指令 */
uint8_t GYRO_INSRUCT_SleepAndWake[5]={0xff, 0xaa, 0x22, 0x01, 0x00} ;
	/* 六轴、九轴算法选择指令 */
uint8_t GYRO_INSRUCT_SixAxis[5]={0xff, 0xaa, 0x24, 0x01, 0x00} ;
uint8_t GYRO_INSRUCT_NineAxis[5]={0xff, 0xaa, 0x24, 0x00, 0x00} ;

/* Private variables ---------------------------------------------------------*/
float GYRO_AngleYawBase ;
float GYRO_AngleYawRelative=0 ;

/* Private function prototypes -----------------------------------------------*/


/**
  * @brief		陀螺仪数据解算
  * @param		GYRO_Data : 输出结果，GYRO_DataTypeDef结构体指针，在Driver层中定义
  * @retval		None
	* @note			关于UART_DataTypeDef类型(即UART2_RX)应该如何传递给函数的问题，目前暂无成熟想法，
	*						暂时通过直接使用BSP层全局变量解决.
	*						Driver层的函数应当在设立的Task层中解决，可以定期解算，也可以使用Semaphore通知解算.
	*						函数直接调用即可给出输出结果，自动根据解算内容调整
  */
void GYRO_GetData(GYRO_DataTypeDef* GYRO_Data) 
{
	uint8_t* ptr=GYRO_RAW_Data.DataBuf ;
	uint8_t temp_cycle = 0 ;
	if(GYRO_VerifyBuf() == GYRO_DATAVERIFY_PASSED)
	{
		while(GYRO_RAW_Data.DataLength-temp_cycle*11>0)
		{
			switch(ptr[temp_cycle*11+1])
			{
				/* 解算加速度 Acceleration :0x51 */
				case 0x51 :
				{
					GYRO_Data->Acceleration.x_Roll=(short)((ptr[temp_cycle*11+3]<<8)|ptr[temp_cycle*11+2])
																					*(16*GYRO_GravityRatio/(float)32768) ;
					GYRO_Data->Acceleration.y_Yaw=(short)((ptr[temp_cycle*11+5]<<8)|ptr[temp_cycle*11+4])
																					*(16*GYRO_GravityRatio/(float)32768) ;
					GYRO_Data->Acceleration.z_Pitch=(short)((ptr[temp_cycle*11+7]<<8)|ptr[temp_cycle*11+6])
																					*(16*GYRO_GravityRatio/(float)32768) ;
					GYRO_Data->Temperature = (short)((ptr[temp_cycle*11+9]<<8)|ptr[temp_cycle*11+8])
																					/(float)100 ;
					GYRO_Data->Acceleration.Flag = 0x01 ;
					break ;
				}
				/* 解算角速度 Angular Velocity :0x52 */
				case 0x52 :
				{
					GYRO_Data->AngularVelocity.x_Roll=(short)((ptr[temp_cycle*11+3]<<8)|ptr[temp_cycle*11+2])
																					*(2000/(float)32768) ;
					GYRO_Data->AngularVelocity.y_Yaw=(short)((ptr[temp_cycle*11+5]<<8)|ptr[temp_cycle*11+4])
																					*(2000/(float)32768) ;
					GYRO_Data->AngularVelocity.z_Pitch=(short)((ptr[temp_cycle*11+7]<<8)|ptr[temp_cycle*11+6])
																					*(2000/(float)32768) ;
					GYRO_Data->Temperature = (short)((ptr[temp_cycle*11+9]<<8)|ptr[temp_cycle*11+8])
																					/(float)100 ;
					GYRO_Data->AngularVelocity.Flag = 0x01 ;
					break ;
				}
				/* 解算角速 Angle :0x53 */
				case 0x53 :
				{
					GYRO_Data->Angle.x_Roll=(short)((ptr[temp_cycle*11+3]<<8)|ptr[temp_cycle*11+2])
																					*(180/(float)32768) ;
					GYRO_Data->Angle.z_Pitch=(short)((ptr[temp_cycle*11+5]<<8)|ptr[temp_cycle*11+4])
																					*(180/(float)32768) ;
					GYRO_Data->Angle.y_Yaw=(short)((ptr[temp_cycle*11+7]<<8)|ptr[temp_cycle*11+6])
																					*(180/(float)32768) ;
					GYRO_Data->Version = (short)((ptr[temp_cycle*11+9]<<8)|ptr[temp_cycle*11+8]) ;
					GYRO_Data->Angle.Flag = 0x01 ;
					break ;
				}
				/* 解算磁场 Magnetic Field :0x54 */
				case 0x54 :
				{
					GYRO_Data->Magnetic.x_Roll=(short)((ptr[temp_cycle*11+3]<<8)|ptr[temp_cycle*11+2]) ;
					GYRO_Data->Magnetic.y_Yaw=(short)((ptr[temp_cycle*11+5]<<8)|ptr[temp_cycle*11+4]) ;
					GYRO_Data->Magnetic.z_Pitch=(short)((ptr[temp_cycle*11+7]<<8)|ptr[temp_cycle*11+6]) ;
					GYRO_Data->Temperature = (short)((ptr[temp_cycle*11+9]<<8)|ptr[temp_cycle*11+8])
																					/(float)100 ;
					GYRO_Data->Magnetic.Flag = 0x01 ;
					break ;
				}
				/* 解算端口 Ports :0x55 */
				case 0x55 :
				{
					GYRO_Data->Ports.d0=(short)((ptr[temp_cycle*11+3]<<8)|ptr[temp_cycle*11+2]) ;
					GYRO_Data->Ports.d1=(short)((ptr[temp_cycle*11+5]<<8)|ptr[temp_cycle*11+4]) ;
					GYRO_Data->Ports.d2=(short)((ptr[temp_cycle*11+7]<<8)|ptr[temp_cycle*11+6]) ;
					GYRO_Data->Ports.d3=(short)((ptr[temp_cycle*11+9]<<8)|ptr[temp_cycle*11+8]) ;
					GYRO_Data->Magnetic.Flag = 0x01 ;
					break ;
				}
				default :
				{
					/* 处理错误 */
					break ;
				}
			}
			temp_cycle++ ;
		}
		GYRO_Data->Flag = 0x01 ;
		GYRO_RAW_Data.Flag = 0x00 ;
	}
	else
	{
		GYRO_Data->Flag = 0x00 ;
		GYRO_RAW_Data.Flag = 0x00 ;
	}
}

/**
  * @brief		陀螺仪数据验证
  * @param		None
  * @retval		GYRO_DATAVERIFY_PASSED:数据校验通过
	*						GYRO_DATAVERIFY_FAILED:数据校验错误
	* @note			None
  */
uint8_t GYRO_VerifyBuf(void) 
{
	uint8_t* ptr=GYRO_RAW_Data.DataBuf ;
	uint8_t temp_cycle = 0 ;
	uint8_t temp_sum=0 ;
	while(GYRO_RAW_Data.DataLength-temp_cycle*11>0)
	{
		/* 检查首字节:0x55 */
		if(ptr[temp_cycle*11] != 0x55)
		{
			return GYRO_DATAVERIFY_FAILED ;
		}
		/* 检查和校验:第[10]位 */
		temp_sum = 0 ;
		for(uint8_t i=0; i<10; i++)
		{
			temp_sum += ptr[temp_cycle*11+i] ;
		}
		if(temp_sum != ptr[temp_cycle*11+10])
		{
			return GYRO_DATAVERIFY_FAILED ;
		}
		temp_cycle++ ;
	}
	return GYRO_DATAVERIFY_PASSED ;
}

/**
  * @brief		陀螺仪配置修改
  * @param		None
  * @retval		None
	* @note			目前主要供调试中使用，完善程度尚不足以用于正式工程
	*						下一步修改计划：通过宏定义、移位、按位或运算实现回传内容、频率的修改
  */
void GYRO_ModifySettings(void) 
{
	/* 解锁指令配置 */
	HAL_UART_Transmit_DMA(&GYRO_UART, GYRO_INSRUCT_UNLOCK, 5) ;
	HAL_Delay(10) ;
	/* 指令配置 */
	HAL_UART_Transmit_DMA(&GYRO_UART, GYRO_INSRUCT_SixAxis, 5) ;
//	HAL_Delay(10) ;
	/* 保存指令配置 */
	HAL_UART_Transmit_DMA(&GYRO_UART, GYRO_INSRUCT_SAVE, 5) ;
//	HAL_Delay(10) ;
}

/**
  * @brief		陀螺仪航向角 YAW 校准
  * @param		GYRO_Data:输入数据，GYRO_DataTypeDef*类型，通过解算得到.
  * @retval		None
	* @note			可能的使用方法：使用信号量，指示其他部分停止读取陀螺仪数据后，调用本函数.
	*						函数原理：不断尝试读取有效陀螺仪数据，达到10个后计算平均值.
	*						
  */
void GYRO_CalibrateYawBase(GYRO_DataTypeDef* GYRO_Data) 
{
	uint8_t temp_cycle=0 ;
	double temp_YawBase=0 ;
	/* 记录十次有效数据 */
	while(temp_cycle < 10)
	{
		if(GYRO_Data->Angle.Flag == 0x01) 
		{
			temp_YawBase += GYRO_Data->Angle.y_Yaw ;
			GYRO_Data->Angle.Flag = 0x00 ;
			temp_cycle++ ;
		}
	}
	GYRO_AngleYawBase = temp_YawBase/10 ;
}

/**
  * @brief		陀螺仪相对航向角 YAW 计算
  * @param		GYRO_Data:输入数据，GYRO_DataTypeDef*类型，通过解算得到.
  * @retval		None
	* @note			可能的使用方法：在Task层中调用.
	*						函数原理：与Base比较，并进行范围调整.
	*						调用函数后，可以保证 GYRO_AngleYawRelative 处于-180~180的范围内，且
	*						GYRO_AngleYawRelative=0 的角度对应于 GYRO_AngleYawBase 实际指向的角度
  */
void GYRO_GetRelativeYawAngle(GYRO_DataTypeDef* GYRO_Data) 
{
	/* 角度归一化 */
	GYRO_AngleYawRelative = GYRO_Data->Angle.y_Yaw - GYRO_AngleYawBase ; 
	if(GYRO_AngleYawBase>=0 && GYRO_AngleYawRelative<-179)	GYRO_AngleYawRelative += 360 ;
	else if(GYRO_AngleYawBase<0 && GYRO_AngleYawRelative>179) GYRO_AngleYawRelative -=360 ;
}
