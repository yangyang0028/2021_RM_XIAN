#include "LED.h"
#include "main.h"
#include "cmsis_os.h"
#include "Holder.h"
#include "Chassis.h"
#include "usart.h"
#include "stdio.h"

//extern ChassisStruct Chassis;

//int fputc(int ch, FILE *f)
//{
//  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
//  return ch;
//}



void LedTask(void const * argument){
  while(1){
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_RESET);
    osDelay(100);
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_SET);
		osDelay(100);
		//printf("w=%lf", (*Chassis.FeedBackCurrent[0]+*Chassis.FeedBackCurrent[1]+*Chassis.FeedBackCurrent[2]+*Chassis.FeedBackCurrent[3])*20/16384.0);
  }
}

