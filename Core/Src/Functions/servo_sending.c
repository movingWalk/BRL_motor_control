#include "servo_sending.h"

#include "stm32f4xx_hal.h"

//**********************************sending***************************//
extern CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef   Tx0Header;
uint32_t              Tx0Mailbox;

extern uint8_t buffer[8];

//CANOpne_sendFrame, Tutorial
void sendFrameEXTId(uint16_t ID, uint8_t* data, uint8_t len){
  
  Tx0Header.ExtId = ID;
  Tx0Header.RTR = CAN_RTR_DATA;
  Tx0Header.IDE = CAN_ID_EXT;
  Tx0Header.DLC = len;
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
  HAL_CAN_AddTxMessage(&hcan1, &Tx0Header, data, &Tx0Mailbox);
  while(HAL_CAN_IsTxMessagePending(&hcan1, Tx0Mailbox) == 1);
}

//Data Sheet
void buffer_append_int32(uint8_t* buffer, int32_t number, uint8_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}


void buffer_append_int16(uint8_t* buffer, int16_t number, uint8_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

// sending protocol
void comm_can_set_duty(uint8_t controller_id, float duty) {
  uint8_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
  sendFrameEXTId(controller_id |((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

void comm_can_set_current(uint8_t controller_id, float current) {
  uint8_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  sendFrameEXTId(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void comm_can_set_cb(uint8_t controller_id, float current) {
  uint8_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  sendFrameEXTId(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm) {
  uint8_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)rpm, &send_index);
  sendFrameEXTId(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

void comm_can_set_pos(uint8_t controller_id, float pos) {
  uint8_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  sendFrameEXTId(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
  uint8_t buffer[1];
  buffer[0]=set_origin_mode;
  sendFrameEXTId(controller_id | ((uint32_t) CAN_PACKET_SET_ORIGIN_HERE << 8), buffer, 1);
}

void comm_can_set_pos_spd(uint8_t controller_id, float pos,int16_t spd, int16_t RPA ) {
  uint8_t send_index = 0;
  uint8_t buffer[8];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  buffer_append_int16(buffer,spd*10, & send_index);
  buffer_append_int16(buffer,RPA*10, & send_index);
  sendFrameEXTId(controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index);
}


//python code: https://tmotorcancontrol.readthedocs.io/en/latest/_modules/TMotorCANControl/servo_can.html#servo_command.__init__

void enter_motor_mode_servo(uint8_t controller_id){
  buffer[0]=0xFF;
  buffer[1]=0xFF;
  buffer[2]=0xFF;
  buffer[3]=0xFF;
  buffer[4]=0xFF;
  buffer[5]=0xFF;
  buffer[6]=0xFF;
  buffer[7]=0xFC;
  sendFrameEXTId(controller_id, buffer, 8);
}

void exit_motor_mode_servo(uint8_t controller_id){
  buffer[0]=0xFF;
  buffer[1]=0xFF;
  buffer[2]=0xFF;
  buffer[3]=0xFF;
  buffer[4]=0xFF;
  buffer[5]=0xFF;
  buffer[6]=0xFF;
  buffer[7]=0xFD;
  sendFrameEXTId(controller_id, buffer, 8);
}
