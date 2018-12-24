#include "bsp_can.h"
#include "driver_chassis.h"
#include "BSPConfig.h"
#include "task_lostcounter.h"
#include "driver_gimbal.h"
#include "driver_feedmotor.h"

static CAN_TxHeaderTypeDef Tx1Message;
static CAN_TxHeaderTypeDef Tx2Message;

void CanInit(void)
{
  UserCan1FilterConfig();													//CAN1:���̵��*4���������*1
  UserCan2FilterConfig();													//CAN2:��̨���*2
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
}

unsigned char UserCan1FilterConfig()
{
	CAN_FilterTypeDef  CAN1_FilerConf;

	CAN1_FilerConf.FilterIdHigh=CAN1_FILTER_ID_FMI2<<5;     //32λID
  CAN1_FilerConf.FilterIdLow=CAN1_FILTER_ID_FMI0<<5;
  CAN1_FilerConf.FilterMaskIdHigh=CAN1_FILTER_ID_FMI3<<5; //32λMASK
  CAN1_FilerConf.FilterMaskIdLow=CAN1_FILTER_ID_FMI1<<5;
  CAN1_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0;//������0������FIFO0
  CAN1_FilerConf.FilterBank=0;          //������0
  CAN1_FilerConf.FilterMode=CAN_FILTERMODE_IDLIST;
  CAN1_FilerConf.FilterScale=CAN_FILTERSCALE_16BIT;
  CAN1_FilerConf.FilterActivation=ENABLE; //�����˲���0
  CAN1_FilerConf.SlaveStartFilterBank=14;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&CAN1_FilerConf)!=HAL_OK) return 1;//�˲�����ʼ��

	__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//FIFO0��Ϣ�����ж�����.	  

	return 0;
}

unsigned char UserCan2FilterConfig()
{
	CAN_FilterTypeDef  CAN2_FilerConf;
	
	CAN2_FilerConf.FilterIdHigh=CAN2_FILTER_ID_FMI2<<5;     //32λID
  CAN2_FilerConf.FilterIdLow=CAN2_FILTER_ID_FMI0<<5;
  CAN2_FilerConf.FilterMaskIdHigh=CAN2_FILTER_ID_FMI3<<5; //32λMASK
  CAN2_FilerConf.FilterMaskIdLow=CAN2_FILTER_ID_FMI1<<5;
  CAN2_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0<<5;//������0������FIFO0
  CAN2_FilerConf.FilterBank=14;          //������0
  CAN2_FilerConf.FilterMode=CAN_FILTERMODE_IDLIST;
  CAN2_FilerConf.FilterScale=CAN_FILTERSCALE_16BIT;
  CAN2_FilerConf.FilterActivation=ENABLE; //�����˲���0
  CAN2_FilerConf.SlaveStartFilterBank=14;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&CAN2_FilerConf)!=HAL_OK) return 1;//�˲�����ʼ��

	__HAL_CAN_ENABLE_IT(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);//FIFO0��Ϣ�����ж�����.	  
	return 0;	
}


#define CAN_RFxR_RFOMx_RELEASE (0x0020U)
#define CAN_RFxR_FULL_FLAG (0x0008U)
#define CAN_RFxR_OVERRUN_FLAG (0x0010U)
#define CAN_RFxR_FMPx_MASK (0x0003U)
#define CAN_IER_FMP0 (0x0002U)

uint8_t Data[8];
CAN_RxHeaderTypeDef RxMessage;
void LL_CAN_Receive(CAN_HandleTypeDef* hcan)
{
	uint32_t tmp1 = 0U, tmp2 = 0U;
	tmp1 = ((hcan->Instance->RF0R & CAN_RFxR_FULL_FLAG) == CAN_RFxR_FULL_FLAG);
	tmp2 = ((hcan->Instance->RF0R & CAN_RFxR_OVERRUN_FLAG) == CAN_RFxR_OVERRUN_FLAG);
  if(tmp1 || tmp2)
		hcan->Instance->RF0R = CAN_RFxR_FULL_FLAG | CAN_RFxR_OVERRUN_FLAG;
	
	tmp1 = ((hcan->Instance->RF0R & CAN_RFxR_FMPx_MASK) != 0U);
  tmp2 = ((hcan->Instance->IER & CAN_IER_FMP0) == CAN_IER_FMP0);
  
  if(tmp1 && tmp2)
  {	
		RxMessage.IDE = (uint8_t)0x04 & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR;
		if (RxMessage.IDE == CAN_ID_STD)
		{
			RxMessage.StdId = 0x000007FFU & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR >> 21U);
		}
		else
		{
			RxMessage.ExtId = 0x1FFFFFFFU & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR >> 3U);
		}
		RxMessage.RTR = (uint8_t)0x02 & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR;
		RxMessage.DLC = (uint8_t)0x0F & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDTR;
		RxMessage.Timestamp = (CAN_RDT0R_TIME & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDTR) >> CAN_RDT0R_TIME_Pos;
		RxMessage.FilterMatchIndex = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDTR >> 8U);
		
		
		Data[0] = (uint8_t)0xFF & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR;
		Data[1] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR >> 8U);
		Data[2] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR >> 16U);
		Data[3] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR >> 24U);
		Data[4] = (uint8_t)0xFF & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR;
		Data[5] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR >> 8U);
		Data[6] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR >> 16U);
		Data[7] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR >> 24U);
		
		hcan->Instance->RF0R = CAN_RFxR_RFOMx_RELEASE;
		HAL_CAN_RxFifo0MsgPendingCallback(hcan);
		
	}
}

extern GimbalMotorStruct	YawMotor,PitchMotor;
extern FeedMotorStruct	 	FeedMotor;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	u8 i;
	if(hcan==&hcan1)
	{
		__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		LostCounterFeed(RxMessage.FilterMatchIndex);
  
		for(i=0;i<8;i++)
		{
			ChassisMotor[RxMessage.FilterMatchIndex].SpeedReceiveMessege[i] = Data[i];
		}
		
	}else if(hcan==&hcan2)
	{
		__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
		switch(RxMessage.FilterMatchIndex)
		{
			case 0: 
				for(i=0;i<8;i++)
				{
					YawMotor.CANReceiveMessege[i] = Data[i];
				}
				LostCounterFeed(GIMBAL_MOTOR_YAW);
				break;
				
			case 1:
				for(i=0;i<8;i++)
				{
					PitchMotor.CANReceiveMessege[i] = Data[i];
				}
				LostCounterFeed(GIMBAL_MOTOR_PITCH);
				break;
				
			case 2:
				for(i=0;i<8;i++)
				{
					FeedMotor.ReceiveMessege[i] = Data[i];
				}
				LostCounterFeed(FEEDMOTOR_LOST_COUNT);
				break;
				
			default:
				break;
		}
	}
}


u8 CAN1_Send_Msg(u8* msg,u8 len)
{
    u16 i=0;
	  uint8_t Data[8];
    Tx1Message.StdId=0x200;        //��׼��ʶ��
    Tx1Message.ExtId=0x12;        //��չ��ʶ��(29λ)
    Tx1Message.IDE=CAN_ID_STD;    //ʹ�ñ�׼֡
    Tx1Message.RTR=CAN_RTR_DATA;  //����֡
    Tx1Message.DLC=len;
    for(i=0;i<len;i++)
			Data[i]=msg[i];
		if(HAL_CAN_AddTxMessage(&hcan1,&Tx1Message,Data,(uint32_t *)CAN_TX_MAILBOX0)!=HAL_OK) return 1;     //����
    return 0;		
}

u8 CAN2_Send_Msg(u8* msg,u8 len)
{
    u16 i=0;
		uint8_t Data[8];
    Tx2Message.StdId=0x1FF;        //��׼��ʶ��
    Tx2Message.ExtId=0x12;        //��չ��ʶ��(29λ)
    Tx2Message.IDE=CAN_ID_STD;    //ʹ�ñ�׼֡
    Tx2Message.RTR=CAN_RTR_DATA;  //����֡
    Tx2Message.DLC=len;
    for(i=0;i<len;i++)
			Data[i]=msg[i];
		if(HAL_CAN_AddTxMessage(&hcan2,&Tx1Message,Data,(uint32_t *)CAN_TX_MAILBOX0)!=HAL_OK) return 1;     //����
    return 0;		
}