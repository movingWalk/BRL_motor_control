#include "servo_receiving.h"

extern float motor_pos;
extern float motor_spd;
extern float motor_cur;
extern int8_t motor_temp;
extern int8_t motor_error;
//can receive protocol

void motor_receive(uint8_t* rx_message)
{
  int16_t pos_int = rx_message[0] << 8 | rx_message[1];
  int16_t spd_int = rx_message[2] << 8 | rx_message[3];
  int16_t cur_int = rx_message[4] << 8 | rx_message[5];
  motor_pos= (float)( pos_int * 0.1f); //motor position
  motor_spd= (float)( spd_int * 10.0f);//motor speed
  motor_cur= (float) ( cur_int * 0.01f);//motor current
  motor_temp= rx_message[6] ;//motor temperature
  motor_error= rx_message[7] ;//motor error mode
}