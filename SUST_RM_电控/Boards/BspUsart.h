#ifndef __BspUsart_H
#define __BspUsart_H

#include "usart.h"

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<7)
#define RC_FRAME_LENGTH 18u




void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle);

HAL_StatusTypeDef UART_Receive_DMA_NoIT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT_IDLE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void HAL_UART_IDLE_IRQHandler(UART_HandleTypeDef *huart);


typedef struct {
	int ch1;	//each ch value from -364 -- +364
	int ch2;
	int ch3;
	int ch4;
	
	int switch_left;	//3 value
	int switch_right;
	
	int16_t x;
	int16_t y;
	int16_t z;
	int press_l;
	int press_r;
	
	uint16_t v;
	
	int LHolder;
	int RHolder;

}RC_Type;

enum{
	Switch_Up = 1,
	Switch_Middle = 3,
	Switch_Down = 2,
};

enum{
	Key_W,
	Key_S,
};

extern RC_Type remote_control;
extern uint32_t  Latest_Remote_Control_Pack_Time ;
void Callback_RC_Handle(RC_Type* rc, uint8_t* buff);

#endif 
