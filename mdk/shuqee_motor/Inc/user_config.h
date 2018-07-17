#ifndef __USER_CONFIG_H
#define __USER_CONFIG_H

#include "stm32f1xx_hal.h"

/* debug mode */
/* #define DEBUG_ENV */

/* three degrees of freedom platform  */
/* #define ENV_3DOF */
/* two degrees of freedom platform  */
/* #define ENV_2DOF */

#define ENV_AIR

#ifdef ENV_3DOF
    /* without sensor */
	#define ENV_NOSENSOR
	/* need to reset the platform */
	#define ENV_RESET
	#define MOTION1_CONFIG_DIR	GPIO_PIN_RESET
	#define MOTION2_CONFIG_DIR	GPIO_PIN_RESET
	#define MOTION3_CONFIG_DIR	GPIO_PIN_RESET
	#define MOTION1_CONFIG_ORIGIN	0
	#define MOTION2_CONFIG_ORIGIN	0
	#define MOTION3_CONFIG_ORIGIN	0
	#define MOTION1_CONFIG_ADJ		10
	#define MOTION2_CONFIG_ADJ		10
	#define MOTION3_CONFIG_ADJ		10
	/* the environment variable of motion-space*/
	#define ENV_SPACE 46
	/* the reload value of timer when speed is in max */
	#define ENV_SPEED_MAX 37
	#define ENV_ACCER     (ENV_SPACE * (uint32_t)256 * (uint32_t)20)
#endif

#ifdef ENV_2DOF
    /* need to reset the platform */
	#define ENV_RESET
	#define MOTION1_CONFIG_DIR	GPIO_PIN_SET
	#define MOTION2_CONFIG_DIR	GPIO_PIN_RESET
	#define MOTION3_CONFIG_DIR	GPIO_PIN_RESET
	#define MOTION1_CONFIG_ORIGIN	0
	#define MOTION2_CONFIG_ORIGIN	0
	#define MOTION3_CONFIG_ORIGIN	0
	#define MOTION1_CONFIG_ADJ		0
	#define MOTION2_CONFIG_ADJ		0
	#define MOTION3_CONFIG_ADJ		0
	/* the environment variable of motion-space*/
	#define ENV_SPACE 25
	/* the reload value of timer when speed is in max */
	#define ENV_SPEED_MAX 37
	#define ENV_ACCER     (ENV_SPACE * (uint32_t)256 * (uint32_t)20)
#endif

#ifdef ENV_AIR
    /* need to reset the platform */
	#define MOTION1_CONFIG_DIR	GPIO_PIN_RESET
	#define MOTION2_CONFIG_DIR	GPIO_PIN_RESET
	#define MOTION3_CONFIG_DIR	GPIO_PIN_RESET
	#define MOTION1_CONFIG_ORIGIN	0
	#define MOTION2_CONFIG_ORIGIN	0
	#define MOTION3_CONFIG_ORIGIN	0
	#define MOTION1_CONFIG_ADJ		0
	#define MOTION2_CONFIG_ADJ		0
	#define MOTION3_CONFIG_ADJ		0
	/* the environment variable of motion-space*/
	#define ENV_SPACE 16
	/* the reload value of timer when speed is in max */
	#define ENV_SPEED_MAX 96
	#define ENV_SPEED_MIN 459
	#define ENV_SPEED_ACCER 5
	#define ENV_ACCER     (ENV_SPACE * (uint32_t)256 * (uint32_t)20)
	/*作为在线调节的标准速度值*/
	#define SPEED_STANDARD 2000000/4096     //毫秒单位(放大1000倍)；
#endif

/* atomic instructions */
#define SAFE(x) do{ \
	__set_PRIMASK(1); \
	x; \
	__set_PRIMASK(0); \
}while(0)

struct high
{
	/* current height of motion */
	int now;
	/* set height of motion */
	int set;
	uint8_t test_bit;
	int last;
};

enum motion_num
{
	MOTION1 = 0,
	MOTION2,
	MOTION3,
	MOTION_COUNT
};

#ifndef ENV_AIR
struct motion_io
{
	GPIO_TypeDef *	dir_port;
	uint16_t				dir_pin;	
	GPIO_TypeDef *	pul_port;
	uint16_t				pul_pin;	
	GPIO_TypeDef *	nup_port;
	uint16_t				nup_pin;
	GPIO_TypeDef *	ndown_port;
	uint16_t				ndown_pin;	
};

struct motion_config
{
	/* set direction of motion */
	GPIO_PinState dir;
	/* origin of motion */
	int origin;
	/* adjustment of height of motion during reset */
	int adj;
};

struct motion_status
{
	/* number of motion */
	enum motion_num index;
	struct high high;	
	/* current direction of motion */
	GPIO_PinState dir;
	struct motion_io io;
	struct motion_config config;
};

struct status
{
	/* the seat number */
	uint8_t id;
	/* sum of the seat who has be sitting  */
	uint8_t seat_num;
	/* flag of enable-seat; platform can move only when (seat_num||seat_enable == true) */
	uint8_t seat_enable;
	/* special effects of seat*/
	uint8_t spb;
	uint8_t uplimit[MOTION_COUNT];
	uint8_t downlimit[MOTION_COUNT];
};
#else
struct motion_io
{
	GPIO_TypeDef *	up_port;
	uint16_t				up_pin;	
	GPIO_TypeDef *	down_port;
	uint16_t				down_pin;	
};

struct motion_pid
{
	int set_point;     //设定目标 Desired Value
    long sum_error;                //误差累计
    double  proportion;         //比例常数 Proportional Cons
    double  integral;           //积分常数 Integral Const
    double  derivative;         //微分常数 Derivative Const
    int last_error;               //Error[-1]
    int prev_error;               //Error[-2]
    double out;                  //输出控制量
};

struct motion_min_begin
{
	int up_origin;
	int down_origin;
	int last_up_origin;
	int last_down_origin;
};	

struct motion_time_record
{
	int up_record;
	int down_record;
	uint8_t flag;
};	

struct speed
{
	int now;
	int set;
	int speed_timecnt;
	int record_up_max;
	int record_down_max;
	uint8_t up_count;
	uint8_t down_count;
	int record_high;
	int record_set;
	int count;
};	

struct motion_status
{
	/* number of motion */
	enum motion_num index;
	struct high high;	
	/* current direction of motion */
	GPIO_PinState dir;
	struct motion_io io;
	struct motion_pid pid;
	struct motion_min_begin min_begin;
	struct motion_time_record time_record;
	struct speed speed;
};

struct status
{
	/* the seat number */
	uint8_t id;
	/* sum of the seat who has be sitting  */
	uint8_t seat_num;
	/* flag of enable-seat; platform can move only when (seat_num||seat_enable == true) */
	uint8_t seat_enable;
	/* special effects of seat*/
	uint8_t spb;
};
#endif

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern CAN_HandleTypeDef hcan;

extern struct motion_status motion[MOTION_COUNT];
extern struct status status;
#endif /* __USER_CONFIG_H */
