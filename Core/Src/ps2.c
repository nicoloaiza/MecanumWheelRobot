#include "ps2.h"
/*********************************************************
**********************************************************/
unsigned short Handkey;
unsigned char Comd[2]={0x01,0x42};
unsigned char Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned short MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
};

int PS2_LX, PS2_LY, PS2_RX, PS2_RY, PS2_KEY;

extern TIM_HandleTypeDef htim8;


void delay_us (unsigned short us)
{
	__HAL_TIM_SET_COUNTER(&htim8,0);
	while (__HAL_TIM_GET_COUNTER(&htim8) < us);
}



#define DELAY_TIME  delay_us(5);


void PS2_Cmd(unsigned char CMD)
{
	volatile unsigned short ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, 1);
		}
		else HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, 0);

		HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, 1);
		DELAY_TIME;
		HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, 0);
		DELAY_TIME;
		HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, 1);
		if(HAL_GPIO_ReadPin(PS2_DATA_GPIO_Port, PS2_DATA_Pin))
			Data[1] = ref|Data[1];
	}
	HAL_Delay(1);
}

unsigned char PS2_RedLight(void)
{
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 0);
	PS2_Cmd(Comd[0]);
	PS2_Cmd(Comd[1]);
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 1);
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}

void PS2_ReadData(void)
{
	volatile unsigned char byte=0;
	volatile unsigned short ref=0x01;
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 0);
	PS2_Cmd(Comd[0]);
	PS2_Cmd(Comd[1]);
	for(byte=2;byte<9;byte++)
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, 1);
			DELAY_TIME;
			HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, 0);
			DELAY_TIME;
			HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, 1);
			if(HAL_GPIO_ReadPin(PS2_DATA_GPIO_Port, PS2_DATA_Pin))
				Data[byte] = ref|Data[byte];
		}
		delay_us(16);
	}
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 1);
}


unsigned char PS2_DataKey()
{
	unsigned char index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];
	for(index=0;index<16;index++)
	{
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;
}


unsigned char PS2_AnologData(unsigned char button)
{
	return Data[button];
}


void PS2_ClearData()
{
	unsigned char a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}

void PS2_Vibration(unsigned char motor1, unsigned char motor2)
{
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 0);
	delay_us(16);
    PS2_Cmd(0x01);
	PS2_Cmd(0x42);
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 1);
	delay_us(16);
}
//short poll
void PS2_ShortPoll(void)
{
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 0);
	delay_us(16);
	PS2_Cmd(0x01);
	PS2_Cmd(0x42);
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 1);
	delay_us(16);
}

void PS2_EnterConfing(void)
{
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 0);
	delay_us(16);
	PS2_Cmd(0x01);
	PS2_Cmd(0x43);
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 1);
	delay_us(16);
}

void PS2_TurnOnAnalogMode(void)
{
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 0);
	PS2_Cmd(0x01);
	PS2_Cmd(0x44);
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0xEE);

	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 1);
	delay_us(16);
}

void PS2_VibrationMode(void)
{
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 0);
	delay_us(16);
	PS2_Cmd(0x01);
	PS2_Cmd(0x4D);
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 1);
	delay_us(16);
}

void PS2_ExitConfing(void)
{
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 0);
	delay_us(16);
	PS2_Cmd(0x01);
	PS2_Cmd(0x43);
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, 1);
	delay_us(16);
}

void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();
	PS2_TurnOnAnalogMode();
	//PS2_VibrationMode();
	PS2_ExitConfing();
}
void PS2_Receive (void)
{
	PS2_KEY=PS2_DataKey();
	PS2_LX=PS2_AnologData(PSS_LX);
	PS2_LY=PS2_AnologData(PSS_LY);
	PS2_RX=PS2_AnologData(PSS_RX);
	PS2_RY=PS2_AnologData(PSS_RY);

}



