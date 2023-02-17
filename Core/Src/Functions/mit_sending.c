#include "mit_sending.h"

#include "stm32f4xx_hal.h"
#include <math.h>


//CAN Sending: CAN tutorial, CANOpen CANOpen_sendFrame 함수 
extern CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef   TxHeader;
uint32_t              TxMailbox;

void sendFrame_std(uint16_t ID, uint8_t* data, uint8_t len){
  
  TxHeader.StdId = ID;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = len;
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);
  while(HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox) == 1);
}



//when sending packet, all the numbers should be converted into integer numbers
//mini-cheetah
int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (unsigned int) ((x-offset)*((float)((1<<bits)-1))/span);
    }

/*
arduino code: https://www.youtube.com/watch?v=UWc_7-8gXeE&t=159s
unsigned int float_to_uint(float x, float x_min, float x_max, int bits){
    float span = x_max - x_min;
    float offset = x_min;
    unsigned int pgg = 0;
    if (bits == 12){
        pgg = (unsigned int) ((x-offset)*4095.0/span);
    }
    if (bits == 14){
        pgg = (unsigned int) ((x-offset)*65535.0/span);
    }
    return pgg;
}
*/

/*
data sheet
int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
	/// Converts a float to an unsigned int, given range and number of bits ///
	float span = x_max - x_min;
	if(x < x_min) x = x_min;
	else if(x > x_max) x = x_max;
	return (int) ((x- x_min)*((float)((1<<bits)/span)));
}
*/


//send routine
// datasheet & arduino
void pack_cmd(uint8_t* buffer, float p_des, float v_des, float kp, float kd, float t_ff){
  
  p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);   
  v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
  kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
  kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
  t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
  /// convert floats to unsigned ints ///
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
  /// pack ints into the can buffer ///
  buffer[0] = p_int>>8; // Position 8 higher
  buffer[1] = p_int&0xFF; // Position 8 lower
  buffer[2] = v_int>>4; // Speed 8 higher
  buffer[3] = ((v_int&0xF)<<4)|(kp_int>>8); //Speed 4 bit lower KP 4bit higher
  buffer[4] = kp_int&0xFF; // KP 8 bit lower
  buffer[5] = kd_int>>4; // Kd 8 bit higher
  buffer[6] = ((kd_int&0xF)<<4)|(kp_int>>8); //KP 4 bit lower torque 4 bit higher
  buffer[7] = t_int&0xFF; // torque 4 bit lower
  
}

/*
Datasheet
void pack_cmd(CANMessage * msg, float p_des, float v_des, float kp, float kd, float t_ff){
	/// limit data to be within bounds ///
	float P_MIN =-95.5;
	float P_MAX =95.5;
	float V_MIN =-30;
	float V_MAX =30;
	float T_MIN =-18;
	float T_MAX =18;
	float Kp_MIN =0;
	float Kp_MAX =500;
	float Kd_MIN =0;
	float Kd_MAX =5;
	float Test_Pos=0.0;
	p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);   // fminf: float, return smaller value / fmaxf: float, return larger value 
	v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
	kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
	kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
	t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
	/// convert floats to unsigned ints ///
	int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
	int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
	int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
	int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
	int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
	/// pack ints into the can buffer ///
	msg->data[0] = p_int>>8; // Position 8 higher
	msg->data[1] = p_int&0xFF; // Position 8 lower
	msg->data[2] = v_int>>4; // Speed 8 higher
	msg->data[3] = ((v_int&0xF)<<4)|(kp_int>>8); //Speed 4 bit lower KP 4bit higher
	msg->data[4] = kp_int&0xFF; // KP 8 bit lower
	msg->data[5] = kd_int>>4; // Kd 8 bit higher
	msg->data[6] = ((kd_int&0xF)<<4)|(kp_int>>8); //KP 4 bit lower torque 4 bit higher
	msg->data[7] = t_int&0xff; // torque 4 bit lower
}
*/

/*
Mini-Cheetah

/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
void unpack_cmd(CANMessage msg, ControllerStruct * controller){
        int p_int = (msg.data[0]<<8)|msg.data[1];
        int v_int = (msg.data[2]<<4)|(msg.data[3]>>4);
        int kp_int = ((msg.data[3]&0xF)<<8)|msg.data[4];
        int kd_int = (msg.data[5]<<4)|(msg.data[6]>>4);
        int t_int = ((msg.data[6]&0xF)<<8)|msg.data[7];
        
        controller->p_des = uint_to_float(p_int, P_MIN, P_MAX, 16);
        controller->v_des = uint_to_float(v_int, V_MIN, V_MAX, 12);
        controller->kp = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
        controller->kd = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
        controller->t_ff = uint_to_float(t_int, T_MIN, T_MAX, 12);
    //printf("Received   ");
    //p
*/