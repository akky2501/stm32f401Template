
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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //タイマ2有効化

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); //SPI2

	//APB1はシステムの半分のクロック?AHBは高速で、84MHzそのまま？
	//TIMに関してAPB1で起動するけど、クロック2倍で動くからそのままのクロックで動くのか？

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
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	
	//Pushswitch setting
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//ここはハード的にPULLUPされてる
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	//SPI2
	//13:sck 14:miso 15:mosi
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


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

/**************************************************************************************/

void OutString(char *s)
{
	while (*s)
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty

		USART_SendData(USART2, *s++); // Send Char
	}
}

/**************************************************************************************/

void Delay(__IO uint32_t nCount) {
	for (; nCount != 0; nCount--)
		;
}


uint8_t SPI2_Transaction(uint8_t Data)
{
	uint16_t RxData;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);		// wait until transmit complete
	SPI_I2S_SendData(SPI2, Data);										// write data to be transmitted to the SPI data register
	Delay(10);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);	// wait until receive complete
	RxData = SPI_I2S_ReceiveData(SPI2);								// return received data from SPI data register
	return (uint8_t)RxData;
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

int main(void)
{
	SystemInit();

	RCC_Configuration();

	GPIO_Configuration();

	USART2_Configuration();


	Delay(0xff);//ごみ値検証用

	OutString("Connected to Nucleo F401RE."); OutString("\n\r");

	//クロック周波数の確認
	char str[150];
	RCC_ClocksTypeDef RCC_ClockFreq;
	RCC_GetClocksFreq(&RCC_ClockFreq);
	//printf("Frequency\r\nSYSCLK:%d\r\nHCLK:%d\r\nPCLK1:%d\r\nPCLK2:%d\r\n", RCC_ClockFreq.SYSCLK_Frequency, RCC_ClockFreq.HCLK_Frequency, RCC_ClockFreq.PCLK1_Frequency, RCC_ClockFreq.PCLK2_Frequency);
	OutString("SYSCLK:");
	itoa(str, RCC_ClockFreq.SYSCLK_Frequency);
	OutString(str); OutString("\n\r");
	OutString("HCLK:");
	itoa(str, RCC_ClockFreq.HCLK_Frequency);
	OutString(str); OutString("\n\r");

	////////////////////////////

	//SPI setting
	/* SPI2	configuration */
	SPI_InitTypeDef SPI_InitStructure = { 0 };
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//通常時のHigh,Low
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//ラッチ
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);


	////////////////////////////
	
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
	// 42MHz*2=84MHz? -> 10kHz -> 10kHz -> 1Hz
	TIM_TimeBaseStructure.TIM_Period = 10000 - 1;//何カウントしたら割り込みするか10kHz-->1Hz
	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;//一カウントするのに何クロック要求するか84MHz-->10kHz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//84MHz / 1 = 84MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

/////////////////////////////
	Delay(0xff);
	char a = SPI2_Transaction(0);
	if(*(char*)&a == -128) OutString("get initial value\n\r");
	char duty = 0;
	while (1){
		
		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 1){	//pull
			//OutString("stop\n\r");
			//duty = 0;
		}
		else{	//push
			duty++;
			uint8_t t = *(uint8_t*)&duty;
			t = SPI2_Transaction(t);
			char dat = *(char*)&t;
			itoa(str,dat);
			OutString("prev duty : ");
			OutString(str);
			OutString("\n\r");
			Delay(0xfff);
			while (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0);
		}

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
	OutString("$");
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


