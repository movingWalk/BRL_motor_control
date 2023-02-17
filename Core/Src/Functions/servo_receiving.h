#ifndef __SERVO_RECEIVING_H__
#define __SERVO_RECEIVING_H__

#include <stdint.h>

typedef enum {
    NO_ERROR = 0,
    OVER_TEMP_FAULT,
    OVER_CURRENT_FAULT,
    OVER_VOLTAGE_FAULT,
    UNDER_VOLTAGE_FAULT,
    ENCODER_FAULT,
    PHASE_CURRENT_UNBALANCE_FAULT,
  } ERROR_CODES;

void motor_receive(uint8_t* rx_message);

#endif