#include "stm32f1xx_hal.h"
#include "user_io.h"
#include "user_can.h"
#include "string.h"
#include "user_uart.h"
#include "user_config.h"

typedef void(*p_fun_t)(void);

typedef struct 
{
	uint8_t data[8];
} can_tx_msg_data_t;

typedef enum 
{
	NM_MSG = 0,
	STATUS_MSG,
	CAN_TX_MAX_NUM
} can_tx_msg_t;

can_tx_msg_data_t can_tx_buf[CAN_TX_MAX_NUM] = {0};

typedef struct
{
	can_msg_id_t msg_id;
	uint8_t cycle;
	uint8_t count;
	p_fun_t can_tx;
} can_tx_item_t;

void can_tx_handle(void);
void can_rx_handle(void);
/**********************CAN的任务内容函数定义******************************/
static void can_tx_nm(void)
{
	//send STATUS_MSG_ID into can bus
//	LED_UP_LIMIT2_TOGGLE();
//	can_send(NM_MSG_ID, can_tx_buf[NM_MSG].data, 8);
}

static void can_tx_status(void)
{
	//send NM_MSG_ID into can bus
//	LED_UP_LIMIT3_TOGGLE();
//	can_send(STATUS_MSG_ID, can_tx_buf[STATUS_MSG].data, 8);
}

/**********************CAN数组形式的任务内容的添加******************************/
can_tx_item_t can_tx_table[CAN_TX_MAX_NUM];

can_msg_id_t tx_msg_id_maping[CAN_TX_MAX_NUM] =
{
	NM_MSG_ID,
	STATUS_MSG_ID
};

uint8_t tx_msg_cycle_init[CAN_TX_MAX_NUM] =
{
	50, /* 0 NM_MSG:500ms */
	10 /* 1 STATUS_MSG:100ms */
};

p_fun_t can_tx_fun_init[CAN_TX_MAX_NUM] =
{
	&can_tx_nm, /* 0 NM_MSG */
	&can_tx_status /* 1 STATUS_MSG */
};

/**********************CAN的任务内容初始化函数******************************/
void can_tx_servet_init(void)
{
	can_tx_msg_t index ;
	for (index = NM_MSG; index < CAN_TX_MAX_NUM; index++)
	{
		can_tx_table[index].msg_id = tx_msg_id_maping[index];
		can_tx_table[index].cycle = tx_msg_cycle_init[index];
		can_tx_table[index].count = 0;
		can_tx_table[index].can_tx = can_tx_fun_init[index];
	}
}

/**********************CAN与TIME产生交集的函数******************************/
void task_can_tx(void)
{
	CAN1->IER|=(1<<1); //确保CAN可以在线热插拔；
//	LED_UP_LIMIT1_TOGGLE();
	can_tx_handle(); 
}

/**********************CAN的任务进程函数******************************/
void can_tx_handle(void)
{
	can_tx_msg_t index ;
	for (index = NM_MSG; index < CAN_TX_MAX_NUM; index++)
	{
		if (can_tx_table[index].count == 0)
		{
			can_tx_table[index].count = can_tx_table[index].cycle;
			can_tx_table[index].can_tx();
		}
		if (can_tx_table[index].count > 0)
		{
			can_tx_table[index].count--;
		}
	}
}

/**********************CAN外部发送数据的调用函数******************************/
void set_status_msg(uint8_t *tx_data)
{
	memcpy(can_tx_buf[STATUS_MSG].data, tx_data, 8);
}

void set_nm_msg(uint8_t *tx_data)
{
	memcpy(can_tx_buf[NM_MSG].data, tx_data, 8);
}


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/********************************CAN接收模块的编写********************************************/
/****↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*******/

typedef struct
{
	uint16_t msg_id;        //接收的ID号
	uint8_t flag;           //置位标志；
	p_fun_t can_rx;        //正常执行程序；
	uint16_t count;        //时间计数；
	uint16_t timeout;     //超时时间；
	p_fun_t timeout_process;     //超时进程处理；
}can_rx_item_t;

can_rx_item_t can_rx_table[CAN_RX_MAX_NUM]={0};

uint16_t msg_id_mapping[CAN_RX_MAX_NUM]=
{
	HIGHT_MSG_ID, 
	SPEED_MSG_ID,
	SP_MSG_ID,
	HEART_BEAT_ID
};

uint8_t can_rx_flag_init[CAN_RX_MAX_NUM]={0};

static void can_rx_hight(void)
{
    //something to do;
}

static void can_rx_speed(void)
{
    //something to do;	
}	

static void can_rx_sp(void)
{
//	LED_DOWN_LIMIT2_TOGGLE();
}	

static void can_rx_hb(void)
{
//	LED_DOWN_LIMIT3_TOGGLE();
}	
p_fun_t can_rx_fun_init[CAN_RX_MAX_NUM]=
{
	&can_rx_hight,
	&can_rx_speed,
	&can_rx_sp,
	&can_rx_hb
};

uint16_t can_rx_count_init[CAN_RX_MAX_NUM]={0};

uint16_t can_rx_timeout_init[CAN_RX_MAX_NUM]=
{
	25,
	25,
	25,
	1000
};

static void can_rx_hight_ot(void)
{
			//something to do;	
}	
static void can_rx_speed_ot(void)
{
	    //something to do;
}	
static void can_rx_sp_ot(void)
{
	    //something to do;
}	
static void can_rx_hb_ot(void)
{
    //something to do;	
}	
p_fun_t can_rx_ot_process[CAN_RX_MAX_NUM]=
{
	&can_rx_hight_ot,
	&can_rx_speed_ot,
	&can_rx_sp_ot,
	&can_rx_hb_ot	
};

void can_rx_init(void)
{
	can_rx_msg_t index;
	for(index = HIGHT_MSG;index < CAN_RX_MAX_NUM;index++)
	{
		can_rx_table[index].msg_id=msg_id_mapping[index];
		can_rx_table[index].flag=0;
		can_rx_table[index].count=0;
		can_rx_table[index].can_rx=can_rx_fun_init[index];
		can_rx_table[index].timeout=can_rx_timeout_init[index];
		can_rx_table[index].timeout_process=can_rx_ot_process[index];
	}
}	
static uint8_t update_flag;
uint8_t can_hb_buff[8]={0,0x01,0x55};

//static uint16_t record_cnt;
void can_rx_handle(void)
{
	uint8_t index;
//	can_rx_msg_t index;
//	for(index = HIGHT_MSG;index < CAN_RX_MAX_NUM;index++)
//	{
//		if(can_rx_table[index].flag==1)
//		{
			/*返回心跳信号*/
			/*can_hb_buff[0]代表座椅地址  can_hb_buff[1]代表心跳信号  can_hb_buff[2]代表验证码*/	
//			if(can_rx_table[HEART_MSG].flag==1)
//			{
//				can_send(HEART_BEAT_ID+status.id,can_hb_buff, 8);
//			}	
//			if(index==3) 
//			{
				SAFE(update_flag=1);
				memcpy(can_rx_buff[0].data,hcan.pRxMsg->Data,8);
//				record_cnt	++;
//	      if(record_cnt>=50)
//				{
//					record_cnt=0;
					for(index=0;index<3;index++)
					{
						//求出当前最新的设定速度值；
						motion[2-index].speed.set=((((can_rx_buff[0].data[index])*ENV_SPACE)-((can_rx_buff[1].data[index])*ENV_SPACE))*1000)/10;    //4906/500ms;
						if(motion[index].speed.set<0)
						motion[2-index].speed.set=-((((can_rx_buff[0].data[index])*ENV_SPACE)-((can_rx_buff[1].data[index])*ENV_SPACE))*1000)/10;    //4906/500ms;
						can_rx_buff[1].data[index]=can_rx_buff[0].data[index];
					}
//				}
			  SAFE(can_or_485=0); 
////			}	
//			can_rx_table[index].can_rx();
//			can_rx_table[index].flag=0;
//			can_rx_table[index].count=0;
//			memcpy(can_rx_buff[index].data,hcan.pRxMsg->Data,8);
//		}	
//		if(can_rx_table[index].flag!=1)
//		{
//			can_rx_table[index].count++;
//		}	
//		if(can_rx_table[index].count>=can_rx_table[index].timeout)
//		{
//			can_rx_table[index].timeout_process();
//		}		
//	}
}

/*获取CAN数据的更新位*/
uint8_t get_update_flag(void)
{
	uint8_t update_byte;
	SAFE(update_byte=update_flag);
	return update_byte;
}	
/*清楚CAN数据更新位*/
void clr_update_flag(void)
{
	SAFE(update_flag=0);
}	
/****************提供给外部的FLAG标志位更改*************************/
void set_can_rx_flag(uint16_t msg_id)
{
	can_rx_msg_t index;
	for(index = HIGHT_MSG;index < CAN_RX_MAX_NUM;index++)
	{
		if(msg_id==msg_id_mapping[index])
		{
			SAFE(can_rx_table[index].flag=1);     //使能对应的FLAG标志位；
		}	
	}
}
///////////////////////////////*提供CAN的外部接口*////////////////////////////////////////////////
/*提取的形参   ID段号  ， 缸号*/

//	HIGHT_MSG_P   ,         //高度ID
//	SPEED_MSG_P ,          //速度ID
//	ENV_SP_P ,            //特效ID	
//	SEAT_ID_P ,          //座椅ID
//	SEAT_SP_P           //环境特效ID

//uint8_t msg_buff_l[8]={0};
//uint8_t a,b,c;
//	get_high_speed_date(HIGHT_MSG_P,msg_buff_l);          //示例；
//	get_high_speed_date(ENV_SP_P,&a);
//	get_high_speed_date(SEAT_ID_P,&b);
//	get_high_speed_date(SEAT_SP_P,&c);

void get_high_speed_date(uint16_t msg_addr,uint8_t * pData)   
{
	switch(msg_addr)
	{
		case HIGHT_MSG_P:
			   memcpy(pData,can_rx_buff[HIGHT_MSG].data,8);
				 break;
		case SPEED_MSG_P:
			   memcpy(pData,can_rx_buff[SPEED_MSG].data,8);
				 break;
		case SEAT_SP_P:	
				 memcpy(pData,(can_rx_buff[SP_MSG].data+1),1); //返回座椅特效  1
				 break;
		case SEAT_ID_P:
				 memcpy(pData,(can_rx_buff[SP_MSG].data+2),1);  //返回座椅ID号；	2
				 break;
		case ENV_SP_P:
				 memcpy(pData,(can_rx_buff[SP_MSG].data),1);  //返回环境特效； 0
		     break;
		default:
			   break;
	}	
}
