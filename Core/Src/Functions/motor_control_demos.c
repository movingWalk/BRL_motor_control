#include "motor_control_demos.h"

#define PI 3.14159265
#define G 9.81
#define KT_ak60_6 0.068 // 0.068Nm/A(from datasheet), 0.087(KT real from Neurobionics)

extern float motor_pos;
extern float current;
extern float duty;
extern uint16_t adcValues[1];

void gravity_compensation_initialize(uint16_t ID){
    enter_motor_mode_servo(ID);

    comm_can_set_origin(ID, 0);
    
    
}

//mass: kg, arm_length: m
void gravity_compensation(uint16_t ID, float arm_length, float mass, float arm_current){

    current = -(((mass*arm_length)*sin(motor_pos*PI/180))/(KT_ak60_6)+arm_current*sin(motor_pos*PI/180));
    comm_can_set_current(ID,current);
    
    /*
    if (fabsf(current)>0.5){
        comm_can_set_current(ID, current);
    }

    else {
        if (0 < motor_pos < 20){
            duty = -0.01*motor_pos/6;
            comm_can_set_duty(ID, duty);
        }
        else if (340 < motor_pos < 360){
            duty = 0.01*(360-motor_pos)/6;
            comm_can_set_duty(ID, duty);
        }
    }
    */
}

/*sin wave generation fixed
count 입력 받아서(timer interrupt마다 +1) period초 주기로 1A를 왔다갔다 (sin wave)
*/
float sin_profile_generate(uint8_t cnt, float amplitude){
    float t = cnt/500.0;
    return  amplitude*(1-cos(t*PI));
}

/* PID Control using ARM PID Library
f32: arm_pid_init_f32, arm_pid_reset_f32, arm_pid_f32
*/
double kp = 2;
double ki = 3;
double kd = 5;

double error;
double lastError;
double cumError, rateError;

void pid_current(uint8_t ID, double input, float desired_val, double elapsedTime){
   error = desired_val - input;
   cumError = cumError + 1/2*(error+lastError) * elapsedTime;
   rateError = (error - lastError)/elapsedTime;
   
   lastError = error;
    
   current = 0.001*(kp*error + ki*cumError + kd*rateError);
   if (current>5) current = 5;
   else if (current < -5) current = -5;
   comm_can_set_current(ID, current);
}