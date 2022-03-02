/**
  ******************************************************************************
  * @file           	Driver_Gyroscope.c
  * @brief          	���ļ�����ά������ WITMOTION ��˾ JY901S ������ģ������.
	*										ʵ��ģ�����ݽ��㣬�����޸�.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.1�״η���
  *
  *	@Contributor			DogeYellow
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Driver_Gyroscope.h"

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private constants ---------------------------------------------------------*/
	/* ����ָ���������ָ��ǰ��Ҫ���͸�ָ��ȴ�10ms��ִ������ָ�� */
uint8_t GYRO_INSRUCT_UNLOCK[5]={0xff, 0xaa, 0x69, 0x88, 0xb5} ;
	/* ����ָ�������д��FLASH����Ҫ�ȴ�100ms��ִ������ָ�� */
uint8_t GYRO_INSRUCT_SAVE[5]={0xff, 0xaa, 0x00, 0x00, 0x00} ;
	/* ��װ��ʽ����ָ�� */
uint8_t GYRO_INSRUCT_VerticalSet[5]={0xff, 0xaa, 0x23, 0x01, 0x00} ;
uint8_t GYRO_INSRUCT_HorizontalSet[5]={0xff, 0xaa, 0x23, 0x00, 0x00} ;
	/* ˯�ߡ�����ָ�� */
uint8_t GYRO_INSRUCT_SleepAndWake[5]={0xff, 0xaa, 0x22, 0x01, 0x00} ;
	/* ���ᡢ�����㷨ѡ��ָ�� */
uint8_t GYRO_INSRUCT_SixAxis[5]={0xff, 0xaa, 0x24, 0x01, 0x00} ;
uint8_t GYRO_INSRUCT_NineAxis[5]={0xff, 0xaa, 0x24, 0x00, 0x00} ;

/* Private variables ---------------------------------------------------------*/
float GYRO_AngleYawBase ;
float GYRO_AngleYawRelative=0 ;

/* Private function prototypes -----------------------------------------------*/


/**
  * @brief		���������ݽ���
  * @param		GYRO_Data : ��������GYRO_DataTypeDef�ṹ��ָ�룬��Driver���ж���
  * @retval		None
	* @note			����UART_DataTypeDef����(��UART2_RX)Ӧ����δ��ݸ����������⣬Ŀǰ���޳����뷨��
	*						��ʱͨ��ֱ��ʹ��BSP��ȫ�ֱ������.
	*						Driver��ĺ���Ӧ����������Task���н�������Զ��ڽ��㣬Ҳ����ʹ��Semaphore֪ͨ����.
	*						����ֱ�ӵ��ü��ɸ������������Զ����ݽ������ݵ���
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
				/* ������ٶ� Acceleration :0x51 */
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
				/* ������ٶ� Angular Velocity :0x52 */
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
				/* ������� Angle :0x53 */
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
				/* ����ų� Magnetic Field :0x54 */
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
				/* ����˿� Ports :0x55 */
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
					/* ������� */
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
  * @brief		������������֤
  * @param		None
  * @retval		GYRO_DATAVERIFY_PASSED:����У��ͨ��
	*						GYRO_DATAVERIFY_FAILED:����У�����
	* @note			None
  */
uint8_t GYRO_VerifyBuf(void) 
{
	uint8_t* ptr=GYRO_RAW_Data.DataBuf ;
	uint8_t temp_cycle = 0 ;
	uint8_t temp_sum=0 ;
	while(GYRO_RAW_Data.DataLength-temp_cycle*11>0)
	{
		/* ������ֽ�:0x55 */
		if(ptr[temp_cycle*11] != 0x55)
		{
			return GYRO_DATAVERIFY_FAILED ;
		}
		/* ����У��:��[10]λ */
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
  * @brief		�����������޸�
  * @param		None
  * @retval		None
	* @note			Ŀǰ��Ҫ��������ʹ�ã����Ƴ̶��в�����������ʽ����
	*						��һ���޸ļƻ���ͨ���궨�塢��λ����λ������ʵ�ֻش����ݡ�Ƶ�ʵ��޸�
  */
void GYRO_ModifySettings(void) 
{
	/* ����ָ������ */
	HAL_UART_Transmit_DMA(&GYRO_UART, GYRO_INSRUCT_UNLOCK, 5) ;
	HAL_Delay(10) ;
	/* ָ������ */
	HAL_UART_Transmit_DMA(&GYRO_UART, GYRO_INSRUCT_SixAxis, 5) ;
//	HAL_Delay(10) ;
	/* ����ָ������ */
	HAL_UART_Transmit_DMA(&GYRO_UART, GYRO_INSRUCT_SAVE, 5) ;
//	HAL_Delay(10) ;
}

/**
  * @brief		�����Ǻ���� YAW У׼
  * @param		GYRO_Data:�������ݣ�GYRO_DataTypeDef*���ͣ�ͨ������õ�.
  * @retval		None
	* @note			���ܵ�ʹ�÷�����ʹ���ź�����ָʾ��������ֹͣ��ȡ���������ݺ󣬵��ñ�����.
	*						����ԭ�����ϳ��Զ�ȡ��Ч���������ݣ��ﵽ10�������ƽ��ֵ.
	*						
  */
void GYRO_CalibrateYawBase(GYRO_DataTypeDef* GYRO_Data) 
{
	uint8_t temp_cycle=0 ;
	double temp_YawBase=0 ;
	/* ��¼ʮ����Ч���� */
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
  * @brief		��������Ժ���� YAW ����
  * @param		GYRO_Data:�������ݣ�GYRO_DataTypeDef*���ͣ�ͨ������õ�.
  * @retval		None
	* @note			���ܵ�ʹ�÷�������Task���е���.
	*						����ԭ����Base�Ƚϣ������з�Χ����.
	*						���ú����󣬿��Ա�֤ GYRO_AngleYawRelative ����-180~180�ķ�Χ�ڣ���
	*						GYRO_AngleYawRelative=0 �ĽǶȶ�Ӧ�� GYRO_AngleYawBase ʵ��ָ��ĽǶ�
  */
void GYRO_GetRelativeYawAngle(GYRO_DataTypeDef* GYRO_Data) 
{
	/* �Ƕȹ�һ�� */
	GYRO_AngleYawRelative = GYRO_Data->Angle.y_Yaw - GYRO_AngleYawBase ; 
	if(GYRO_AngleYawBase>=0 && GYRO_AngleYawRelative<-179)	GYRO_AngleYawRelative += 360 ;
	else if(GYRO_AngleYawBase<0 && GYRO_AngleYawRelative>179) GYRO_AngleYawRelative -=360 ;
}
