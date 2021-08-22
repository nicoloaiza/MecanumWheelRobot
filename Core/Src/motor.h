#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "main.h"

#define PWMA   TIM8->CCR4
#define PWMB   TIM8->CCR3
#define PWMC   TIM8->CCR2
#define PWMD   TIM8->CCR1

#define Set_MotorPWMA(x)	TIM_SetCompare3(TIM8, x);
#define INB1   PBout(4)
#define INB2   PBout(5)

#define Set_MotorPWMB(x)	TIM_SetCompare4(TIM8, x);
#define INA2   PDout(2)
#define INA1   PCout(12)

#define Set_MotorPWMC(x)	TIM_SetCompare1(TIM8, x);
#define IND1   PBout(0)
#define IND2   PBout(1)

#define Set_MotorPWMD(x)	TIM_SetCompare2(TIM8, x);
#define INC2   PCout(5)
#define INC1   PCout(4)

#define EN     PAin(12)

enum Direction{Forward, Backward};
enum Motor{Front_Right, Front_Left, Rear_Right, Rear_Left};

void SetMotorDirection(enum Direction direction, enum Motor motor);
void StopMotor(enum Motor motor);
void StopFrontRightMotor();
void StopFrontLeftMotor();
void StopRearRightMotor();
void StopRearLeftMotor();
void StopMotor(enum Motor motor);
void StopMotors();
void MoveForward();
void MoveBackward();
void MoveLeft();
void MoveRight();
void MoveForwardLeft();
void MoveForwardRight();
void MoveBackwardLeft();
void MoveBackwardRight();
void MoveRightRotation();
void MoveLeftRotation();
void TestMotor();
void MoveFrontRightForward();
void MoveFrontRightBackward();
void MoveFrontLeftForward();
void MoveFrontLeftBackward();
void MoveRearRightForward();
void MoveRearRightBackward();
void MoveRearLeftForward();
void MoveRearLeftBackward();
//void MiniBalance_Motor_Init(void);
#endif
