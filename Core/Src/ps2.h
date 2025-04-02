#ifndef __PS2_H
#define __PS2_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include "main.h"

#define DI PCin(2)

extern int PS2_LX, PS2_LY, PS2_RX, PS2_RY, PS2_KEY;


//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

//#define WHAMMY_BAR		8

//These are stick values
#define PSS_RX 5
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

extern unsigned char Data[9];
extern unsigned short MASK[16];
extern unsigned short Handkey;

void PS2_Init(void);
unsigned char PS2_RedLight(void);
void PS2_ReadData(void);
void PS2_Cmd(unsigned char CMD);
unsigned char PS2_DataKey(void);
unsigned char PS2_AnologData(unsigned char button);
void PS2_ClearData(void);
void PS2_Vibration(unsigned char motor1, unsigned char motor2);

void PS2_EnterConfing(void);
void PS2_TurnOnAnalogMode(void);
void PS2_VibrationMode(void);
void PS2_ExitConfing(void);
void PS2_SetInit(void);
void PS2_Receive (void);
#endif





