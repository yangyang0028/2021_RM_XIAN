#include "BspCan.h"
#include "stm32f4xx_hal.h"

moto_info_t motor_info[MOTOR_MAX_NUM];
uint16_t can_cnt;

void can_user_init(void)
{
    CAN_FilterTypeDef can_filter_st; //CAN滤波器
	
    can_filter_st.FilterActivation = ENABLE; //激活该过滤器
	
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK; //设置过滤器组模式
	
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; //设置过滤器位宽
	
    can_filter_st.FilterIdHigh = 0x0000; // 过滤器验证码ID高16位，只能填０到０ｘFFFF　中间的数值
	
    can_filter_st.FilterIdLow = 0x0000; // 过滤器ID低16位，只能填０到０ｘFFFF　中间的数值
	
    can_filter_st.FilterMaskIdHigh = 0x0000; // 过滤器掩码ID高16位，只能填０到０ｘFFFF中间的数值
	
    can_filter_st.FilterMaskIdLow = 0x0000; // 过滤器掩码ID低16位，只能填０到０ｘFFFF中间的数值 
	
    can_filter_st.FilterBank = 0; //使用过滤器编号。使用一个CAN，则可选 0~13；使用两个CAN可选 0~27
	
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0; //关联的过滤器 0 到 FIFO0，值为 CAN_RX_FIFO0
	
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st); //初始化 CAN 的滤波器相关参数
		
    HAL_CAN_Start(&hcan1);//启动CAN
		
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //使能中断


    can_filter_st.SlaveStartFilterBank = 14; //设置 CAN 起始存储区
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data
  if ((rx_header.StdId >= FEEDBACK_ID_BASE)
   && (rx_header.StdId <  FEEDBACK_ID_BASE + MOTOR_MAX_NUM))                  // judge the can id
  {
    can_cnt ++;
    uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE;                  // get motor index by can_id
    motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[index].temp           =   rx_data[6];
		
  }
  if (can_cnt == 500){
    can_cnt = 0;
  }
}
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void set_motor_voltage_MG6020(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = 0x1ff;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
}

/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void set_motor_voltage_C620(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = 0x200;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
}
