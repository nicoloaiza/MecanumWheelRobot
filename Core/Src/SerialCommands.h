
#include "stm32f1xx_hal.h"
#include "string.h"
#include "main.h"
#include <stdbool.h>

#define RxBuf_SIZE 10
#define MainBuf_SIZE 20

extern bool isSerialCommandAvailable;

enum CommandType {
	Stop,
	ForwardMovement,
	BackwardMovement,
	LeftMovement,
	RightMovement,
	ForwardLeftMovement,
	ForwardRightMovement,
	BackwardLeftMovement,
	BackwardRightMovement,
	RightRotationMovement,
	LeftRotationMovement,
	FrontLeftForward,
	FrontLeftBackward,
	FrontRightForward,
	FrontRightBackward,
	RearLeftForward,
	RearLeftBackward,
	RearRightForward,
	RearRightBackward,
	TestMotors,
	ReadMPU6050,
	PrintEncoders
};
struct Command
{
	enum CommandType commandType;
    int value;
};

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void StartReceivingSerialCommands(UART_HandleTypeDef *huart);
struct Command GetNextCommand();
