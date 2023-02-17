#ifndef __MIT_RECEIVING_H__
#define __MIT_RECEIVING_H__

#include <stdint.h>


float uint_to_float(int x_int, float x_min, float x_max, int bits);

void unpack_reply(uint8_t* buffer);

#endif