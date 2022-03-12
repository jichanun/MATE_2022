/**
  ******************************************************************************
  * @file            Driver_Servo.c
  * @brief           本文件定义舵机的相关功能与实现.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.1首次发布
  *
  *	@Contributor			DogeYellow
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "driver_dataglove.h"

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/



/* Private variables ---------------------------------------------------------*/
 /*数据手套各部位定义*/
 //（大臂俯仰角）
uint8_t Angle_x1_HH;
uint8_t Angle_x1_H;
uint8_t Angle_x1_L;
uint8_t Angle_x1_LL;

//（大臂横滚角）
uint8_t Angle_y1_HH;
uint8_t Angle_y1_H;
uint8_t Angle_y1_L;
uint8_t Angle_y1_LL;

//（大臂偏航角）
uint8_t Angle_z1_HH;
uint8_t Angle_z1_H;
uint8_t Angle_z1_L;
uint8_t Angle_z1_LL;

//（小臂俯仰角）
uint8_t Angle_x2_HH;
uint8_t Angle_x2_H;
uint8_t Angle_x2_L;
uint8_t Angle_x2_LL;

//（小臂横滚角）
uint8_t Angle_y2_HH;
uint8_t Angle_y2_H;
uint8_t Angle_y2_L;
uint8_t Angle_y2_LL;

//（小臂偏航角）
uint8_t Angle_z2_HH;
uint8_t Angle_z2_H;
uint8_t Angle_z2_L;
uint8_t Angle_z2_LL;

//（手套四元数（Q1，Q2，Q3，Q4排列））
uint8_t handQ1_H;
uint8_t handQ1_L;
uint8_t handQ2_H;
uint8_t handQ2_L;
uint8_t handQ3_H;
uint8_t handQ3_L;
uint8_t handQ4_H;
uint8_t handQ4_L;

//（手套俯仰角）
uint8_t hand_x_H;
uint8_t hand_x_L;

//（手套横滚角）
uint8_t hand_y_H;
uint8_t hand_y_L;

//（手套偏航角）
uint8_t hand_z_H;
uint8_t hand_z_L;

//（手指弯曲度）
//uint8_t finger1;
//uint8_t finger2;
//uint8_t finger3;
//uint8_t finger4;
//uint8_t finger5;

/*输出数据*/

/* Private constants ---------------------------------------------------------*/
#define DutyIDLE 1500

/* Private function prototypes -----------------------------------------------*/


/**
  * @brief		数据手套数据解算函数
  * @param		Angle : SEVO_DutyTypeDef结构体指针，由Task层控制
  * @retval		None
	* @note			尚未验证数据从串口接收后能否正确传递到函数内部
  */
void ReadData(DTGL_DataTypeDef* Data)//从缓冲拉取数据并解算成角度
{
	/*a =( USART_RX_BUF[2]<<24 | USART_RX_BUF[3]<<16 | USART_RX_BUF[4]<<8 | USART_RX_BUF[5]) /1000;        
		a=((short)((USART_RX_BUF[35]<<8)| USART_RX_BUF[34]))/32768.0*180;      */
	if(DTGL_RAW_Data.Flag==1&&DTGL_RAW_Data.DataBuf[0]==0xaa&&DTGL_RAW_Data.DataBuf[1]==0xaa
			&&DTGL_RAW_Data.DataBuf[45]==0xbb&&DTGL_RAW_Data.DataBuf[46]==0xbb)//检验帧头帧尾	
	{
		//（大臂俯仰角）
		Angle_x1_HH=DTGL_RAW_Data.DataBuf[2];
		Angle_x1_H=DTGL_RAW_Data.DataBuf[3];
		Angle_x1_L=DTGL_RAW_Data.DataBuf[4];
		Angle_x1_LL=DTGL_RAW_Data.DataBuf[5];
		Data->Angle_x1=( Angle_x1_HH<<24 | Angle_x1_H<<16 | Angle_x1_L<<8 | Angle_x1_LL) /1000;
		//（大臂横滚角）
		Angle_y1_HH=DTGL_RAW_Data.DataBuf[6];
		Angle_y1_H=DTGL_RAW_Data.DataBuf[7];
		Angle_y1_L=DTGL_RAW_Data.DataBuf[8]; 
		Angle_y1_LL=DTGL_RAW_Data.DataBuf[9];
		Data->Angle_y1=( Angle_y1_HH<<24 | Angle_y1_H<<16 | Angle_y1_L<<8 | Angle_y1_LL) /1000;
		//（大臂偏航角）
		Angle_z1_HH=DTGL_RAW_Data.DataBuf[10];
		Angle_z1_H=DTGL_RAW_Data.DataBuf[11];
		Angle_z1_L=DTGL_RAW_Data.DataBuf[12];
		Angle_z1_LL=DTGL_RAW_Data.DataBuf[13];
		Data->Angle_z1=( Angle_z1_HH<<24 | Angle_z1_H<<16 | Angle_z1_L<<8 | Angle_z1_LL) /1000;
		//（小臂俯仰角）
		Angle_x2_HH=DTGL_RAW_Data.DataBuf[14];
		Angle_x2_H=DTGL_RAW_Data.DataBuf[15];
		Angle_x2_L=DTGL_RAW_Data.DataBuf[16];
		Angle_x2_LL=DTGL_RAW_Data.DataBuf[17];
		Data->Angle_x2=( Angle_x2_HH<<24 | Angle_x2_H<<16 | Angle_x2_L<<8 | Angle_x2_LL) /1000;
		//（小臂横滚角）
		Angle_y2_HH=DTGL_RAW_Data.DataBuf[18];
		Angle_y2_H=DTGL_RAW_Data.DataBuf[19];
		Angle_y2_L=DTGL_RAW_Data.DataBuf[20];
		Angle_y2_LL=DTGL_RAW_Data.DataBuf[21];
		Data->Angle_y1=( Angle_y2_HH<<24 | Angle_y2_H<<16 | Angle_y2_L<<8 | Angle_y2_LL) /1000;
		//（小臂偏航角）
		Angle_z2_HH=DTGL_RAW_Data.DataBuf[22];
		Angle_z2_H=DTGL_RAW_Data.DataBuf[23];
		Angle_z2_L=DTGL_RAW_Data.DataBuf[24];
		Angle_z2_LL=DTGL_RAW_Data.DataBuf[25];
		Data->Angle_z2=( Angle_z2_HH<<24 | Angle_z2_H<<16 | Angle_z2_L<<8 | Angle_z2_LL) /1000;
		//（手套四元数（Q1，Q2，Q3，Q4排列））解算方式不确定
		handQ1_H=DTGL_RAW_Data.DataBuf[26];
		handQ1_L=DTGL_RAW_Data.DataBuf[27];
		Data->handQ1=((short)((handQ1_L<<8)| handQ1_H))/32768.0*180;

		handQ2_H=DTGL_RAW_Data.DataBuf[28];
		handQ2_L=DTGL_RAW_Data.DataBuf[29];
		Data->handQ2=((short)((handQ2_L<<8)| handQ2_H))/32768.0*180;

		handQ3_H=DTGL_RAW_Data.DataBuf[30];
		handQ3_L=DTGL_RAW_Data.DataBuf[31];
		Data->handQ3=((short)((handQ3_L<<8)| handQ3_H))/32768.0*180;

		handQ4_H=DTGL_RAW_Data.DataBuf[32];
		handQ4_L=DTGL_RAW_Data.DataBuf[33];
		Data->handQ4=((short)((handQ4_L<<8)| handQ4_H))/32768.0*180;

		//（手套俯仰角）
		hand_x_H=DTGL_RAW_Data.DataBuf[34];
		hand_x_L=DTGL_RAW_Data.DataBuf[35];
		Data->hand_x=((short)((hand_x_L<<8)| hand_x_H))/32768.0*180;
		//（手套横滚角）
		hand_y_H=DTGL_RAW_Data.DataBuf[36];
		hand_y_L=DTGL_RAW_Data.DataBuf[37];
		Data->hand_y=((short)((hand_y_L<<8)| hand_y_H))/32768.0*180;
		//（手套偏航角）
		hand_z_H=DTGL_RAW_Data.DataBuf[38];
		hand_z_L=DTGL_RAW_Data.DataBuf[39];
		Data->hand_z=((short)((hand_z_L<<8)| hand_z_H))/32768.0*180;
		//（手指弯曲度）
		Data->finger1=DTGL_RAW_Data.DataBuf[40];
		Data->finger2=DTGL_RAW_Data.DataBuf[41];
		Data->finger3=DTGL_RAW_Data.DataBuf[42];
		Data->finger4=DTGL_RAW_Data.DataBuf[43];
		Data->finger5=DTGL_RAW_Data.DataBuf[44];

		DTGL_RAW_Data.Flag=0;
	}
}

