#include "user_can.h"
#include "user_config.h"
#include "user_time.h"
#include <string.h>

CanTxMsgTypeDef txmessage;
CanRxMsgTypeDef rxmessage;
can_rx_data can_rx_buff[CAN_RX_MAX_NUM]={0}; 
static void can_txconfig()
{
  hcan.pTxMsg=&txmessage;
  hcan.pTxMsg->StdId=0x006;      //���û���ID��
	hcan.pTxMsg->IDE=CAN_ID_STD;   //����Ϊ��׼��ʽ��
	hcan.pTxMsg->RTR=CAN_RTR_DATA; //����Ϊ����֡��
	hcan.pTxMsg->DLC=8;            //�������ݳ���Ϊ8���ֽڣ�
  hcan.pTxMsg->Data[0]='C';
	hcan.pTxMsg->Data[1]='A';
	hcan.pTxMsg->Data[2]='N';
	hcan.pTxMsg->Data[3]=' ';
	hcan.pTxMsg->Data[4]='I'; 
	hcan.pTxMsg->Data[5]='S';
	hcan.pTxMsg->Data[6]='O';
	hcan.pTxMsg->Data[7]='K';
}
/*32λ�б�ģʽ*/
static void can_rxconfig(uint8_t filter_num,uint16_t id_list)
{
	CAN_FilterConfTypeDef sFilterConfig;
	hcan.pRxMsg = &rxmessage;
	sFilterConfig.BankNumber=14;
	sFilterConfig.FilterActivation=ENABLE;
	sFilterConfig.FilterFIFOAssignment=0;
	sFilterConfig.FilterIdHigh=(id_list)<<5;
	sFilterConfig.FilterIdLow=0x0000;
	sFilterConfig.FilterMaskIdHigh=(id_list)<<5;
	sFilterConfig.FilterMaskIdLow=0x0000;

	sFilterConfig.FilterMode=CAN_FILTERMODE_IDLIST;
	sFilterConfig.FilterNumber=filter_num;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;

	HAL_CAN_ConfigFilter(&hcan,&sFilterConfig);	
}	

/*32λ����ģʽ*/
static void can_scale32_idmask(uint8_t filter_num)  
{  
	uint16_t      mask,num,tmp,i;  
  CAN_FilterConfTypeDef  sFilterConfig;
  uint32_t stdidarray[3]={HIGHT_MSG_ID,SPEED_MSG_ID,SP_MSG_ID};  
	for(i=0;i<3;i++)
	{
		stdidarray[i]=HIGHT_MSG_ID+i;
	}		    
  sFilterConfig.FilterNumber = filter_num;               //ʹ�ù����� filter_num  
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;     //����Ϊ����ģʽ  
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;    //����Ϊ32λ��  
  sFilterConfig.FilterIdHigh =(stdidarray[0]<<5);     //��֤���������Ϊstdidarray[]����������һ��������ʹ��stdidarray[0]��Ϊ��֤��  
  sFilterConfig.FilterIdLow =0;  
    
  mask =0x7ff;                      //���濪ʼ����������  
  num =sizeof(stdidarray)/sizeof(stdidarray[0]);  
  for(i =0; i<num; i++)      //������λstdidarray[]���������г�Ա��ͬ����  
  {  
    tmp =stdidarray[i] ^ (~stdidarray[0]);  //���������Ա���0����Ա����ͬ�����  
    mask &=tmp;  
  }  
  sFilterConfig.FilterMaskIdHigh =(mask<<5);  
  sFilterConfig.FilterMaskIdLow =0|0x02;        //ֻ��������֡  
    
  sFilterConfig.FilterFIFOAssignment = 0;       //����ͨ��������֡���뵽FIFO0��  
  sFilterConfig.FilterActivation = ENABLE;  
  sFilterConfig.BankNumber = 14;  
    
  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)  
  {  
    Error_Handler();  
  }  
} 

void user_can_init(void)
{ can_txconfig();
	can_rxconfig(0,HEART_BEAT_ID);
	can_scale32_idmask(1);  
	CAN1->MCR|=(1<<6);
 	HAL_CAN_Receive_IT(&hcan,CAN_FIFO0);
	memset((void *)&can_rx_buff,0,sizeof(can_rx_buff));
}

/**
  * @brief  Transmits a CAN frame message.
  * @param  dest_addr: pointer to which dest_adrr  
  * @param  *data: Pointer to data buffer  
  * @param  len: Amount of data to be send  ,the rang of  0-8;
  * @retval HAL status
  */

void can_send(uint16_t msg_id, uint8_t *data, uint16_t len)
{   
	if(msg_id>=0x7ff)
	{
		msg_id=0x7ff;
	}	
	if(len>=8)
	{
		len=8;
	}	
	hcan.pTxMsg->StdId=msg_id; /*����Ҫ�������ݵ�Ŀ���ַ*/
	hcan.pTxMsg->Data[0]=status.id;
	hcan.pTxMsg->Data[1]=data[1];
	hcan.pTxMsg->Data[2]=data[2];
	hcan.pTxMsg->Data[3]=data[3];
	hcan.pTxMsg->Data[4]=data[4];
	hcan.pTxMsg->Data[5]=data[5];
	hcan.pTxMsg->Data[6]=data[6];
	hcan.pTxMsg->Data[7]=data[7];	
	hcan.pTxMsg->DLC=len;
	if(HAL_CAN_Transmit(&hcan, 1)==HAL_OK)
	{
		; /* do nothing */
	} 
	else     
	{ 
		;/*to do*/
	}	
	CAN1->IER|=(1<<1);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{   
	/*�������յ�����ʲô����*/
	//set_can_rx_flag(hcan->pRxMsg->StdId);
//	if (hcan->pRxMsg->StdId == HIGHT_MSG_ID)
//	{
//		can_rx_handle();
//	}
	can_rx_handle();
	CAN1->IER|=(1<<1);
	HAL_CAN_Receive_IT(hcan,CAN_FIFO0);
}


