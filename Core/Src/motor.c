#include "motor.h"

void SetFrontRightMotorDirection(enum Direction direction)
{
	if (direction == Forward) {
		HAL_GPIO_WritePin(GPIOD, F_AIN1_Pin, 1);
		HAL_GPIO_WritePin(GPIOC, F_AIN2_Pin, 0);
	} else {
		HAL_GPIO_WritePin(GPIOD, F_AIN1_Pin, 0);
		HAL_GPIO_WritePin(GPIOC, F_AIN2_Pin, 1);
	}
}
void SetFrontLeftMotorDirection(enum Direction direction)
{
	if (direction == Forward) {
		HAL_GPIO_WritePin(GPIOB, F_BIN1_Pin, 1);
		HAL_GPIO_WritePin(GPIOB, F_BIN2_Pin, 0);
	} else {
		HAL_GPIO_WritePin(GPIOB, F_BIN1_Pin, 0);
		HAL_GPIO_WritePin(GPIOB, F_BIN2_Pin, 1);
	}
}
void SetRearRightMotorDirection(enum Direction direction)
{
	if (direction == Forward) {
		HAL_GPIO_WritePin(GPIOB, R_BIN1_Pin, 1);
		HAL_GPIO_WritePin(GPIOB, R_BIN2_Pin, 0);
	} else {
		HAL_GPIO_WritePin(GPIOB, R_BIN1_Pin, 0);
		HAL_GPIO_WritePin(GPIOB, R_BIN2_Pin, 1);
	}
}
void SetRearLeftMotorDirection(enum Direction direction)
{
	if (direction == Forward) {
		HAL_GPIO_WritePin(GPIOC, R_AIN1_Pin, 1);
		HAL_GPIO_WritePin(GPIOC, R_AIN2_Pin, 0);
	} else {
		HAL_GPIO_WritePin(GPIOC, R_AIN1_Pin, 0);
		HAL_GPIO_WritePin(GPIOC, R_AIN2_Pin, 1);
	}
}

void SetMotorDirection(enum Direction direction, enum Motor motor)
{
	switch ( motor )
	{
		case Front_Right:
	    	SetFrontRightMotorDirection(direction);
	        break;
	    case Front_Left:
	    	SetFrontLeftMotorDirection(direction);
	        break;
	    case Rear_Right:
	    	SetRearRightMotorDirection(direction);
	        break;
	    case Rear_Left:
	    	SetRearLeftMotorDirection(direction);
	        break;
	}
}

void StopFrontRightMotor() {
	HAL_GPIO_WritePin(GPIOD, F_AIN1_Pin, 0);
	HAL_GPIO_WritePin(GPIOC, F_AIN2_Pin, 0);
}

void StopFrontLeftMotor() {
	HAL_GPIO_WritePin(GPIOB, F_BIN1_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, F_BIN2_Pin, 0);
}

void StopRearRightMotor() {
	HAL_GPIO_WritePin(GPIOB, R_BIN1_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, R_BIN2_Pin, 0);
}

void StopRearLeftMotor() {
	HAL_GPIO_WritePin(GPIOC, R_AIN1_Pin, 0);
	HAL_GPIO_WritePin(GPIOC, R_AIN2_Pin, 0);
}

void StopMotor(enum Motor motor)
{
	switch ( motor )
	{
	    case Front_Right:
	    	StopFrontRightMotor();
	        break;
	    case Front_Left:
	    	StopFrontLeftMotor();
	        break;
	    case Rear_Right:
	    	StopRearRightMotor();
	        break;
	    case Rear_Left:
	    	StopRearLeftMotor();
	        break;
	}
}

void StopMotors()
{
	StopFrontRightMotor();
	StopFrontLeftMotor();
	StopRearRightMotor();
	StopRearLeftMotor();
}

void MoveFrontRightForward()
{
	StopMotors();
	SetFrontRightMotorDirection(Forward);
}

void MoveFrontRightBackward()
{
	StopMotors();
	SetFrontRightMotorDirection(Backward);
}

void MoveFrontLeftForward()
{
	StopMotors();
	SetFrontLeftMotorDirection(Forward);
}

void MoveFrontLeftBackward()
{
	StopMotors();
	SetFrontLeftMotorDirection(Backward);
}

void MoveRearRightForward()
{
	StopMotors();
	SetRearRightMotorDirection(Forward);
}

void MoveRearRightBackward()
{
	StopMotors();
	SetRearRightMotorDirection(Backward);
}

void MoveRearLeftForward()
{
	StopMotors();
	SetRearLeftMotorDirection(Forward);
}

void MoveRearLeftBackward()
{
	StopMotors();
	SetRearLeftMotorDirection(Backward);
}

void MoveForward()
{
	SetFrontRightMotorDirection(Forward);
	SetFrontLeftMotorDirection(Forward);
	SetRearRightMotorDirection(Forward);
	SetRearLeftMotorDirection(Forward);
}

void MoveBackward()
{
	SetFrontRightMotorDirection(Backward);
	SetFrontLeftMotorDirection(Backward);
	SetRearRightMotorDirection(Backward);
	SetRearLeftMotorDirection(Backward);
}

void MoveLeft()
{
	SetFrontRightMotorDirection(Forward);
	SetFrontLeftMotorDirection(Backward);
	SetRearRightMotorDirection(Backward);
	SetRearLeftMotorDirection(Forward);
}

void MoveRight()
{
	SetFrontRightMotorDirection(Backward);
	SetFrontLeftMotorDirection(Forward);
	SetRearRightMotorDirection(Forward);
	SetRearLeftMotorDirection(Backward);
}

void MoveForwardLeft()
{
	StopMotors();
	SetFrontRightMotorDirection(Forward);
	SetRearLeftMotorDirection(Forward);
}

void MoveForwardRight()
{
	StopMotors();
	SetFrontLeftMotorDirection(Forward);
	SetRearRightMotorDirection(Forward);
}

void MoveBackwardLeft()
{
	StopMotors();
	SetFrontLeftMotorDirection(Backward);
	SetRearRightMotorDirection(Backward);
}

void MoveBackwardRight()
{
	StopMotors();
	SetFrontRightMotorDirection(Backward);
	SetRearLeftMotorDirection(Backward);
}

void MoveRightRotation()
{
	SetFrontRightMotorDirection(Backward);
	SetFrontLeftMotorDirection(Forward);
	SetRearRightMotorDirection(Backward);
	SetRearLeftMotorDirection(Forward);
}

void MoveLeftRotation()
{
	SetFrontRightMotorDirection(Forward);
	SetFrontLeftMotorDirection(Backward);
	SetRearRightMotorDirection(Forward);
	SetRearLeftMotorDirection(Backward);
}

void TestMotor()
{
	MoveForward();
	HAL_Delay(2000);
	StopMotors();
	HAL_Delay(2000);
	MoveBackward();
	HAL_Delay(2000);
	StopMotors();
	HAL_Delay(2000);
	MoveLeft();
	HAL_Delay(2000);
	StopMotors();
	HAL_Delay(2000);
	MoveRight();
	HAL_Delay(2000);
	StopMotors();
	HAL_Delay(2000);
	MoveForwardLeft();
	HAL_Delay(2000);
	StopMotors();
	HAL_Delay(2000);
	MoveForwardRight();
	HAL_Delay(2000);
	StopMotors();
	HAL_Delay(2000);
	MoveBackwardLeft();
	HAL_Delay(2000);
	StopMotors();
	HAL_Delay(2000);
	MoveBackwardRight();
	HAL_Delay(2000);
	StopMotors();
	HAL_Delay(2000);
	MoveRightRotation();
	HAL_Delay(2000);
	StopMotors();
	HAL_Delay(2000);
	MoveLeftRotation();
	HAL_Delay(2000);
	StopMotors();
	HAL_Delay(2000);
}
