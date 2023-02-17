#include "extra_motor_function.h"
#include "mit_sending.h"

extern float p_in;
extern float v_in;
extern float kp_in;
extern float kd_in;
extern float t_in;

extern uint8_t TxData[8];
//*********************************Extra Motor Function********************************//
//arduino code
void pull_up(){
  p_in=0.0f;
  v_in=0.0f;
  kp_in = 0.0f;
  kd_in = 0.0f;
  t_in =0.0f;
  
  pack_cmd(TxData, p_in, v_in, kp_in, kd_in, t_in);
  sendFrame_std(1, TxData, 8);
}

void enter_motor_mode(){
  TxData[0]=0xFF;
  TxData[1]=0xFF;
  TxData[2]=0xFF;
  TxData[3]=0xFF;
  TxData[4]=0xFF;
  TxData[5]=0xFF;
  TxData[6]=0xFF;
  TxData[7]=0xFC;

  sendFrame_std(1, TxData, 8);
}

void exit_motor_mode(){
  TxData[0]=0xFF;
  TxData[1]=0xFF;
  TxData[2]=0xFF;
  TxData[3]=0xFF;
  TxData[4]=0xFF;
  TxData[5]=0xFF;
  TxData[6]=0xFF;
  TxData[7]=0xFD;

  sendFrame_std(1, TxData, 8);
}

void set_zero(){
  TxData[0]=0xFF;
  TxData[1]=0xFF;
  TxData[2]=0xFF;
  TxData[3]=0xFF;
  TxData[4]=0xFF;
  TxData[5]=0xFF;
  TxData[6]=0xFF;
  TxData[7]=0xFE;
  
  sendFrame_std(1, TxData, 8);
}


/*
Mini Cheetah

#define REST_MODE           0
#define CALIBRATION_MODE    1
#define MOTOR_MODE          2
#define SETUP_MODE          4
#define ENCODER_MODE        5
#define INIT_TEMP_MODE      6

CAN          can(PB_8, PB_9, 1000000);      // CAN Rx pin name, CAN Tx pin name
CANMessage   rxMsg;
CANMessage   txMsg;

volatile int count = 0;
volatile int state = REST_MODE;
volatile int state_change;

void onMsgReceived() {
    //msgAvailable = true;
    //printf("%d\n\r", rxMsg.id);
    can.read(rxMsg);  
    if((rxMsg.id == CAN_ID)){
        controller.timeout = 0;
        if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) & (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFC))){
            state = MOTOR_MODE;
            state_change = 1;
            }
        else if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFD))){
            state = REST_MODE;
            state_change = 1;
            gpio.led->write(0);; 
            }
        else if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFE))){
            spi.ZeroPosition();
            }
        else if(state == MOTOR_MODE){
            unpack_cmd(rxMsg, &controller);
            }
        pack_reply(&txMsg, controller.theta_mech, controller.dtheta_mech, controller.i_q_filt*KT_OUT);
        can.write(txMsg);
        }
    
}
*/