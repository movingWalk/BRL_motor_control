#ifndef __MOTOR_CONTROL_DEMOS_H__
#define __MOTOR_CONTROL_DEMOS_H__

#include "servo_sending.h"
#include "servo_receiving.h"
#include <math.h>

extern double PI;
extern double G;
extern double KT_ak60_6;

void gravity_compensation_initialize(uint16_t ID);
void gravity_compensation(uint16_t ID, float arm_length, float mass, float arm_moment);
float sin_profile_generate(uint8_t cnt, float amplitude);
void pid_current(uint8_t ID, double input, float desired_val, double elapsedTime);

#endif