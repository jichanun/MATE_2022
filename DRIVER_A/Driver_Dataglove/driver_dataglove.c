/**
  ******************************************************************************
  * @file            Driver_Servo.c
  * @brief           ���ļ�����������ع�����ʵ��.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.1�״η���
  *
  *	@Contributor			DogeYellow
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "driver_dataglove.h"

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/



/* Private variables ---------------------------------------------------------*/
 /*�������׸���λ����*/
 //����۸����ǣ�
uint8_t Angle_x1_HH;
uint8_t Angle_x1_H;
uint8_t Angle_x1_L;
uint8_t Angle_x1_LL;

//����ۺ���ǣ�
uint8_t Angle_y1_HH;
uint8_t Angle_y1_H;
uint8_t Angle_y1_L;
uint8_t Angle_y1_LL;

//�����ƫ���ǣ�
uint8_t Angle_z1_HH;
uint8_t Angle_z1_H;
uint8_t Angle_z1_L;
uint8_t Angle_z1_LL;

//��С�۸����ǣ�
uint8_t Angle_x2_HH;
uint8_t Angle_x2_H;
uint8_t Angle_x2_L;
uint8_t Angle_x2_LL;

//��С�ۺ���ǣ�
uint8_t Angle_y2_HH;
uint8_t Angle_y2_H;
uint8_t Angle_y2_L;
uint8_t Angle_y2_LL;

//��С��ƫ���ǣ�
uint8_t Angle_z2_HH;
uint8_t Angle_z2_H;
uint8_t Angle_z2_L;
uint8_t Angle_z2_LL;

//��������Ԫ����Q1��Q2��Q3��Q4���У���
uint8_t handQ1_H;
uint8_t handQ1_L;
uint8_t handQ2_H;
uint8_t handQ2_L;
uint8_t handQ3_H;
uint8_t handQ3_L;
uint8_t handQ4_H;
uint8_t handQ4_L;

//�����׸����ǣ�
uint8_t hand_x_H;
uint8_t hand_x_L;

//�����׺���ǣ�
uint8_t hand_y_H;
uint8_t hand_y_L;

//������ƫ���ǣ�
uint8_t hand_z_H;
uint8_t hand_z_L;

//����ָ�����ȣ�
uint8_t finger1;
uint8_t finger2;
uint8_t finger3;
uint8_t finger4;
uint8_t finger5;

/*�������*/
float Angle_x1;
float Angle_y1;
float Angle_z1;
float Angle_x2;
float Angle_y2;
float Angle_z2;
float handQ1;
float handQ2;
float handQ3;
float handQ4;
float hand_x;
float hand_y;
float hand_z;

/* Private constants ---------------------------------------------------------*/
#define DutyIDLE 1500

/* Private function prototypes -----------------------------------------------*/


/**
  * @brief		�����������ݽ��㺯��
  * @param		Angle : SEVO_DutyTypeDef�ṹ��ָ�룬��Task�����
  * @retval		None
	* @note			���ڼƻ���ư�ȫģʽ���
  */
	void ReadData(void)//�ӻ�����ȡ���ݲ�����ɽǶ�
	{
		/*a =( USART_RX_BUF[2]<<24 | USART_RX_BUF[3]<<16 | USART_RX_BUF[4]<<8 | USART_RX_BUF[5]) /1000;        
		  a=((short)((USART_RX_BUF[35]<<8)| USART_RX_BUF[34]))/32768.0*180;      */
	if(UART_RX2.Flag==1&&UART_RX2.DataBuf[0]==0xaa&&UART_RX2.DataBuf[1]==0xaa&&UART_RX2.DataBuf[45]==0xbb&&UART_RX2.DataBuf[46]==0xbb)//����֡ͷ֡β	
	{//����۸����ǣ�
Angle_x1_HH=UART_RX2.DataBuf[2];
Angle_x1_H=UART_RX2.DataBuf[3];
Angle_x1_L=UART_RX2.DataBuf[4];
Angle_x1_LL=UART_RX2.DataBuf[5];
Angle_x1=( Angle_x1_HH<<24 | Angle_x1_H<<16 | Angle_x1_L<<8 | Angle_x1_LL) /1000;
//����ۺ���ǣ�
Angle_y1_HH=UART_RX2.DataBuf[6];
Angle_y1_H=UART_RX2.DataBuf[7];
Angle_y1_L=UART_RX2.DataBuf[8]; 
Angle_y1_LL=UART_RX2.DataBuf[9];
Angle_y1=( Angle_y1_HH<<24 | Angle_y1_H<<16 | Angle_y1_L<<8 | Angle_y1_LL) /1000;
//�����ƫ���ǣ�
Angle_z1_HH=UART_RX2.DataBuf[10];
Angle_z1_H=UART_RX2.DataBuf[11];
Angle_z1_L=UART_RX2.DataBuf[12];
Angle_z1_LL=UART_RX2.DataBuf[13];
Angle_z1=( Angle_z1_HH<<24 | Angle_z1_H<<16 | Angle_z1_L<<8 | Angle_z1_LL) /1000;
//��С�۸����ǣ�
Angle_x2_HH=UART_RX2.DataBuf[14];
Angle_x2_H=UART_RX2.DataBuf[15];
Angle_x2_L=UART_RX2.DataBuf[16];
Angle_x2_LL=UART_RX2.DataBuf[17];
Angle_x2=( Angle_x2_HH<<24 | Angle_x2_H<<16 | Angle_x2_L<<8 | Angle_x2_LL) /1000;
//��С�ۺ���ǣ�
Angle_y2_HH=UART_RX2.DataBuf[18];
Angle_y2_H=UART_RX2.DataBuf[19];
Angle_y2_L=UART_RX2.DataBuf[20];
Angle_y2_LL=UART_RX2.DataBuf[21];
Angle_y1=( Angle_y2_HH<<24 | Angle_y2_H<<16 | Angle_y2_L<<8 | Angle_y2_LL) /1000;
//��С��ƫ���ǣ�
Angle_z2_HH=UART_RX2.DataBuf[22];
Angle_z2_H=UART_RX2.DataBuf[23];
Angle_z2_L=UART_RX2.DataBuf[24];
Angle_z2_LL=UART_RX2.DataBuf[25];
Angle_z2=( Angle_z2_HH<<24 | Angle_z2_H<<16 | Angle_z2_L<<8 | Angle_z2_LL) /1000;
//��������Ԫ����Q1��Q2��Q3��Q4���У������㷽ʽ��ȷ��
handQ1_H=UART_RX2.DataBuf[26];
handQ1_L=UART_RX2.DataBuf[27];
handQ1=((short)((handQ1_L<<8)| handQ1_H))/32768.0*180;

handQ2_H=UART_RX2.DataBuf[28];
handQ2_L=UART_RX2.DataBuf[29];
handQ2=((short)((handQ2_L<<8)| handQ2_H))/32768.0*180;

handQ3_H=UART_RX2.DataBuf[30];
handQ3_L=UART_RX2.DataBuf[31];
handQ3=((short)((handQ3_L<<8)| handQ3_H))/32768.0*180;

handQ4_H=UART_RX2.DataBuf[32];
handQ4_L=UART_RX2.DataBuf[33];
handQ4=((short)((handQ4_L<<8)| handQ4_H))/32768.0*180;

//�����׸����ǣ�
hand_x_H=UART_RX2.DataBuf[34];
hand_x_L=UART_RX2.DataBuf[35];
hand_x=((short)((hand_x_L<<8)| hand_x_H))/32768.0*180;
//�����׺���ǣ�
hand_y_H=UART_RX2.DataBuf[36];
hand_y_L=UART_RX2.DataBuf[37];
hand_y=((short)((hand_y_L<<8)| hand_y_H))/32768.0*180;
//������ƫ���ǣ�
hand_z_H=UART_RX2.DataBuf[38];
hand_z_L=UART_RX2.DataBuf[39];
hand_z=((short)((hand_z_L<<8)| hand_z_H))/32768.0*180;
//����ָ�����ȣ�
finger1=UART_RX2.DataBuf[40];
finger2=UART_RX2.DataBuf[41];
finger3=UART_RX2.DataBuf[42];
finger4=UART_RX2.DataBuf[43];
finger5=UART_RX2.DataBuf[44];

UART_RX2.Flag=0;
}
	}


	