
// STM32 USART2 (Tx PA.2, Rx PA.3) STM32F401RE NUCLEO - sourcer32@gmail.com

#include "stm32f4xx.h"

/**************************************************************************************/

void RCC_Configuration(void)
{
	/* --------------------------- System Clocks Configuration -----------------*/
	/* USART2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* GPIO clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

	//I2C1 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
}

/**************************************************************************************/

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*-------------------------- GPIO Configuration ----------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // PA.2 USART2_TX, PA.3 USART2_RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect USART pins to AF */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	//userLED & GPIO setting
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5/*led*/;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Pushswitch setting
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//SCL,SDA setting
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);//SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);//SDA
	
}

/**************************************************************************************/

void USART2_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;

	/* USARTx configuration ------------------------------------------------------*/
	/* USARTx configured as follow:
	- BaudRate = X baud
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure);

	USART_Cmd(USART2, ENABLE);
}

void I2C_Configuration(){
	I2C_InitTypeDef I2C_InitStructure = {0};
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 200000;//100kHz
	I2C_InitStructure.I2C_OwnAddress1 = 0;//自身のアドレス?

	I2C_DeInit(I2C1);
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}

/**************************************************************************************/

void OutString(char *s)
{
	while (*s)
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty

		USART_SendData(USART2, *s++); // Send Char
	}
}

void OutWord(int16_t x)
{
	for (int i = 0; i < 2; i++)
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty

		USART_SendData(USART2, ((uint16_t)x >> (i * 8)) & 0x00ff); // Send Char
	}
}

void dummyResetI2C(void){//動作不能のスレーブをダミークロックによって救済する
	;
}


void sendByteI2C(uint8_t module_address, uint8_t register_address, uint8_t data){
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);
	OutString("+");
	//start flag
	I2C_GenerateSTART(I2C1,ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);
	OutString("+");

	//send address
	I2C_Send7bitAddress(I2C1, module_address, I2C_Direction_Transmitter);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR);
	OutString("+");

	//send register_address
	I2C_SendData(I2C1, register_address);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR);
	OutString("+");

	//send data
	I2C_SendData(I2C1, data);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR);
	OutString("+");

	//stop flag
	I2C_GenerateSTOP(I2C1,ENABLE);
}

uint8_t recieveByteI2C(uint8_t module_address, uint8_t register_address){
	//I2C_AcknowledgeConfig(I2C1, DISABLE);
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);

	// Generate Start Condition
	I2C_GenerateSTART(I2C1, ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);

	// Send slave address
	I2C_Send7bitAddress(I2C1, module_address, I2C_Direction_Transmitter);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR); // Wait for SACK

	// Transfer register address
	I2C_SendData(I2C1, register_address);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR); // Wait for SACK

	// Generate Start Condition
	I2C_GenerateSTART(I2C1, ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);

	// Send slave address
	I2C_Send7bitAddress(I2C1, module_address, I2C_Direction_Receiver);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == ERROR);


	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) == ERROR); // Wait for SACK

	uint8_t data = I2C_ReceiveData(I2C1);

	I2C_AcknowledgeConfig(I2C1, ENABLE);

	return data;
}


uint16_t recieveWordI2C(uint8_t module_address, uint8_t register_address){
	uint16_t d = (uint16_t)(recieveByteI2C(module_address, register_address))<<8;
	d |= recieveByteI2C(module_address, register_address + 1);

	return d;
}

/*
uint16_t recieveWordI2C(uint8_t module_address, uint8_t register_address){
	//I2C_AcknowledgeConfig(I2C1, DISABLE);
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);

	// Generate Start Condition
	I2C_GenerateSTART(I2C1, ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);

	// Send slave address
	I2C_Send7bitAddress(I2C1, module_address, I2C_Direction_Transmitter);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR);

	// Transfer register address
	I2C_SendData(I2C1, register_address);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR);

	// Generate Start Condition
	I2C_GenerateSTART(I2C1, ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);

	// Send slave address
	I2C_Send7bitAddress(I2C1, module_address, I2C_Direction_Receiver);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == ERROR);

	uint16_t data = I2C_ReceiveData(I2C1);


	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) == ERROR);

	data |= ((uint16_t)I2C_ReceiveData(I2C1)) << 8;

	I2C_AcknowledgeConfig(I2C1, ENABLE);
	
	return data;
}*/

/**************************************************************************************/

void Delay(__IO uint32_t nCount) {
	for (; nCount != 0; nCount--)
		;
}
void itoa(char* str, int i){
	int n = 0;
	bool neg = false;
	if (i < 0){
		neg = true;
		i = -i;
	}
	do{
		str[n] = i % 10 + '0';
		i /= 10;
		n++;
	} while (i);
	if (neg) str[n++] = '-';
	str[n] = '\0';

	for (int j = 0; j != n / 2; j++){
		char tmp = str[j];
		str[j] = str[n - j - 1];
		str[n - j - 1] = tmp;
	}
}

void resetMPU6050(void){
	sendByteI2C(0b1101000 << 1, 0x6b, 0b10000000);//device reset
	sendByteI2C(0b1101000 << 1, 0x6b, 0b00000000);//cycle off
	Delay(0xff);

	sendByteI2C(0b1101000 << 1, 0x1b, 0b00010000);//gyro 1000
	sendByteI2C(0b1101000 << 1, 0x1c, 0b00001000);//acc 4
	//sendByteI2C(0b1101000 << 1, 0x68, 0b00000111);//acc gy tmp reset

	
}

struct SensorData{
	struct vector{
		int16_t x, y, z;
	};

	vector gyro;
	vector accel;
	int16_t temp;
};

SensorData getMPU6050(void){
	SensorData d;
	uint16_t t;
	t = recieveWordI2C(0b1101000 << 1, 0x3b); d.accel.x = *(int16_t*)&t;
	t = recieveWordI2C(0b1101000 << 1, 0x3d); d.accel.y = *(int16_t*)&t;
	t = recieveWordI2C(0b1101000 << 1, 0x3f); d.accel.z = *(int16_t*)&t;

	t = recieveWordI2C(0b1101000 << 1, 0x43); d.gyro.x = *(int16_t*)&t;
	t = recieveWordI2C(0b1101000 << 1, 0x45); d.gyro.y = *(int16_t*)&t;
	t = recieveWordI2C(0b1101000 << 1, 0x47); d.gyro.z = *(int16_t*)&t;

	t = recieveWordI2C(0b1101000 << 1, 0x41); d.temp = *(int16_t*)&t;

	return d;
}

int main(void)
{
	SystemInit();

	RCC_Configuration();

	GPIO_Configuration();

	USART2_Configuration();

	I2C_Configuration();

	Delay(0xff);//ごみ値検証用

//	OutString("Connected to Nucleo F401RE."); OutString("\n\r");

	//クロック周波数の確認
	char str[150];
	RCC_ClocksTypeDef RCC_ClockFreq;
	RCC_GetClocksFreq(&RCC_ClockFreq);
	//printf("Frequency\r\nSYSCLK:%d\r\nHCLK:%d\r\nPCLK1:%d\r\nPCLK2:%d\r\n", RCC_ClockFreq.SYSCLK_Frequency, RCC_ClockFreq.HCLK_Frequency, RCC_ClockFreq.PCLK1_Frequency, RCC_ClockFreq.PCLK2_Frequency);
//	OutString("SYSCLK:");
	itoa(str, RCC_ClockFreq.SYSCLK_Frequency);
//	OutString(str); OutString("\n\r");
//	OutString("HCLK:");
	itoa(str, RCC_ClockFreq.HCLK_Frequency);
//	OutString(str); OutString("\n\r");

	////////////////////////////
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //タイマ有効化
	
	NVIC_InitTypeDef NVIC_InitStructure = {0};
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	// Enable the TIM2 gloabal Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

//	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = { 0 };
	// 84MHz -> MHz -> kHz -> 2Hz
	TIM_TimeBaseStructure.TIM_Period = 5000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	// 設定を反映
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	// 割り込み許可...割り込み要因のORを引数に入れる
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	// タイマ2起動
	TIM_Cmd(TIM2, ENABLE);

	/////////////////////////////

	//dummyResetI2C();
	resetMPU6050();


//	OutString("recieve\n\r");

	uint8_t dat = recieveByteI2C(0b1101000 << 1, 0x75);//Who am I ?
	//dat = 0x68 = 104
//	OutString("I'm ");
	itoa(str, dat);
//	OutString(str); OutString(".\n\r");
	
	dat = recieveByteI2C(0b1101000 << 1, 0x1b);
//	OutString("[0x1b] = ");
	itoa(str, dat);
//	OutString(str); OutString(".\n\r");

	dat = recieveByteI2C(0b1101000 << 1, 0x1c);
//	OutString("[0x1c] = ");
	itoa(str, dat);
//	OutString(str); OutString(".\n\r");
	/*
	sendByteI2C(0b1101000 << 1, 0x6c, 0b00000000);

	dat = recieveByteI2C(0b1101000 << 1, 0x6c);
	OutString("[0x6c] = ");
	itoa(str, dat);
	OutString(str); OutString(".\n\r");*/
	
	while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); // Wait for Char
	//Data = USART_ReceiveData(USART2);
	long s = 0;
	while (1){
		SensorData d = getMPU6050();
		OutWord(d.accel.z);
		
		//OutString("ax:"); itoa(str, d.accel.x); OutString(str); OutString(",");
		//OutString("ay:"); itoa(str, d.accel.y); OutString(str); OutString(",");
		//OutString("az:"); itoa(str, d.accel.z); OutString(str); OutString(",");
		//OutString("gx:"); itoa(str, d.gyro.x); OutString(str); OutString(",");
		//OutString("gy:"); itoa(str, d.gyro.y); OutString(str); OutString(",");
		//OutString("gz:"); itoa(str, d.gyro.z); OutString(str); OutString(",");
		//OutString("t:"); itoa(str, d.temp); OutString(str); OutString("\n\r");
		
		/*if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 1){	//pull
			s += d.gyro.z +47;
			OutString("s:"); itoa(str, s); OutString(str); OutString(",");
			OutString("deg:"); itoa(str, s>0?s/131:-(-s/131)); OutString(str); OutString("\n\r");
		}
		else	//push
			s = 0;
		*/
	}
	
	OutString("Exit!");//ありえない
}


void blink(void){
	// 今回は簡単なので直に書く。ここから関数へジャンプするのもあり。

	static char led;

	if (led)
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		led = 0;

	}
	else{
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
		led = 1;
	}
}


//Interrupt Function for TIM2
extern "C" void TIM2_IRQHandler(void)
{
	//OutString("$");
	// グローバル割り込みから個別要因へジャンプするためにフラグチェックする
	// 今回は要因が1つしかないけど、通常はこういうスタイルで分岐する
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		// 割り込み保留ビット(=割り込み要因フラグ)をクリア
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
		blink();

	}
}



/**************************************************************************************/

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	while (1)
	{
	}
}
#endif

/**************************************************************************************/

/**
* @brief  Decrements the TimingDelay variable.
* @param  None
* @retval None
*/
static __IO uint32_t uwTimingDelay;
void TimingDelay_Decrement(void)
{
	if (uwTimingDelay != 0x00)
	{
		uwTimingDelay--;
	}
}


