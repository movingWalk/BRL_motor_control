#ifndef __MIT_SENDING_H__
#define __MIT_SENDING_H__

#include <stdint.h>

//**************************motor configuration*************************//
//datasheet
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define Kp_MIN 0.0f
#define Kp_MAX 500.0f
#define Kd_MIN 0.0f
#define Kd_MAX 5.0f
#define T_MIN -15.0f
#define T_MAX 15.0f

extern void sendFrame_std(uint16_t ID, uint8_t* data, uint8_t len);

int float_to_uint(float x, float x_min, float x_max, int bits);

void pack_cmd(uint8_t* buffer, float p_des, float v_des, float kp, float kd, float t_ff);



#endif