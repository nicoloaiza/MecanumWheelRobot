#include "SerialCommands.h"
#include "stdio.h"

extern DMA_HandleTypeDef hdma_uart4_rx;

uint8_t MainBuf[MainBuf_SIZE];
uint8_t RxBuffer[1];
uint8_t RxCommand[10];
uint16_t currentPosition = 0;
bool isSerialCommandAvailable;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == UART4) {
		MainBuf[currentPosition] = RxBuffer[0];
		if (MainBuf[currentPosition] == '\n') {
			for(int i = 0; i < 10 ;i++) {
				if(i<= currentPosition) {
					RxCommand[i] = MainBuf[i];
				} else {
					RxCommand[i] = 0;
				}
			}
			currentPosition = 0;
			isSerialCommandAvailable = true;
		} else {
			currentPosition++;
		}

		StartReceivingSerialCommands(huart);
	}
}

void StartReceivingSerialCommands(UART_HandleTypeDef *huart) {
	HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*) RxBuffer, 1);
	__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
}

int GetCommandValue(uint8_t* commandValue) {
	uint8_t valueArray[4];
	memcpy( valueArray, commandValue + 1, 4);
	int value = 0;
	sscanf((char*)valueArray, "%d", &value);
	return value;
}

struct Command GetNextCommand() {
	struct Command command;
	switch ( RxCommand[0] )
	{
		case 'W':
			command.commandType = ForwardMovement;
			command.value = GetCommandValue(RxCommand);
	        break;
	    case 'Z':
			command.commandType = BackwardMovement;
			command.value = GetCommandValue(RxCommand);
	        break;
	    case 'A':
			command.commandType = LeftMovement;
			command.value = GetCommandValue(RxCommand);
	        break;
	    case 'S':
			command.commandType = RightMovement;
			command.value = GetCommandValue(RxCommand);
	        break;
	    case 'T':
			command.value = GetCommandValue(RxCommand);
			command.commandType = command.value == 1 ? FrontLeftForward : FrontLeftBackward;
	        break;
	    case 'Y':
			command.value = GetCommandValue(RxCommand);
			command.commandType = command.value == 1 ? FrontRightForward : FrontRightBackward;
	        break;
	    case 'U':
			command.value = GetCommandValue(RxCommand);
			command.commandType = command.value == 1 ? RearLeftForward : RearLeftBackward;
	        break;
	    case 'I':
			command.value = GetCommandValue(RxCommand);
			command.commandType = command.value == 1 ? RearRightForward : RearRightBackward;
	        break;
	    case 'D':
			command.commandType = Stop;
			command.value = GetCommandValue(RxCommand);
	        break;
	    case 'G':
			command.commandType = ReadMPU6050;
			command.value = GetCommandValue(RxCommand);
	        break;
	    case 'P':
			command.commandType = PrintEncoders;
			command.value = GetCommandValue(RxCommand);
	        break;
	    case 'M':
			command.commandType = TestMotors;
			command.value = GetCommandValue(RxCommand);
	        break;
	}
	for(int i = 0; i < 10 ;i++) {
		RxCommand[i] = 0;
	}
	isSerialCommandAvailable = false;
	return command;
}

