#ifndef __BSP_CAN
#define __BSP_CAN
#include "can.h"
#include "main.h"

#define FEEDBACK_ID_BASE      0x200
#define MOTOR_MAX_NUM         8

typedef struct
{
    uint16_t can_id;
    int16_t  set_voltage;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  temp;
}moto_info_t;

void can_user_init(void);
void set_motor_voltage_MG6020(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_motor_voltage_C620(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
#endif
