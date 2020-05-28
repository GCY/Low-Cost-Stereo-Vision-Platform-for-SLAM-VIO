#ifndef __DEFINE__
#define __DEFINE__

//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#define LEFT_CAMERA 1
//#define RIGHT_CAMERA 1

#include "stdint.h"

const unsigned int RED_LED = 33;
const unsigned int FLASH_LED = 4;

#ifdef LEFT_CAMERA
const unsigned int LASER1 = 2;
const unsigned int LASER2 = 14;
const unsigned int IR = 12;
#elif RIGHT_CAMERA
const unsigned int M1 = 12;
const unsigned int M2 = 2;
const unsigned int SHUTTER = 14;
#endif

extern volatile unsigned int M1_POS;
extern volatile unsigned int M2_POS;

extern unsigned int ToF;

extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;
extern double roll, pitch, yaw;

#endif
