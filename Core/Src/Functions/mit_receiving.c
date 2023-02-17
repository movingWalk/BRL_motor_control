#include "mit_receiving.h"
#include "mit_sending.h"

extern float motor_cur;
extern float motor_pos;
extern float motor_spd;

//*********************************CAN Receiving********************************//
float uint_to_float(int x_int, float x_min, float x_max, int bits){ 
    //Mini Cheetah
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

/*
Arduino

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits){

    float span = x_max - x_min;
    float offset = x_min;
    float pgg = 0;
    if (bits = 12){
        pgg = ((float)x_int)*span / 4095.0 + offset;
    }
    if (bits = 16){
        pgg = ((float)x_int)*span / 65535.0 + offset;
    }
    return pgg
}
*/

//receive routine
void unpack_reply(uint8_t* buffer){
  /// unpack ints from can buffer ///
  //data sheet
  int id = buffer[0]; //??ID
  int p_int = (buffer[1]<<8)|buffer[2]; //Motor position data
  int v_int = (buffer[3]<<4)|(buffer[4]>>4); // Motor speed data
  int t_int = ((buffer[4]&0xF)<<8)|buffer[5]; // Motor torque data
  /// convert ints to floats ///
  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float t = uint_to_float(t_int, T_MIN, T_MAX, 12);
  if(id == 1){
    motor_pos = p; //Read the corresponding data according to the ID code
    motor_spd = v;
    motor_cur = t;
  }
}

/*
Mini Cheetah

/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void pack_reply(CANMessage *msg, float p, float v, float t){
    int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(t, -T_MAX, T_MAX, 12);
    msg->data[0] = CAN_ID;
    msg->data[1] = p_int>>8;
    msg->data[2] = p_int&0xFF;
    msg->data[3] = v_int>>4;
    msg->data[4] = ((v_int&0xF)<<4) + (t_int>>8);
    msg->data[5] = t_int&0xFF;
    }
*/