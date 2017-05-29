
// STM32 USART2 (Tx PA.2, Rx PA.3) STM32F401RE NUCLEO - sourcer32@gmail.com

#include "stm32f4xx.h"

/**************************************************************************************/

void RCC_Configuration(void)
{
	/* --------------------------- System Clocks Configuration -----------------*/
	/* USART2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
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

	//userLED setting
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
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

void EchoLoop(void){
	while (1) // Don't want to exit
	{
		uint16_t Data;

		while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); // Wait for Char

		Data = USART_ReceiveData(USART2); // Collect Char

		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty

		USART_SendData(USART2, Data); // Echo Char
	}
}

void itoa(char* str, int i){
	int n = 0;
	do{
		str[n] = i % 10 + '0';
		i /= 10;
		n++;
	} while (i);
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

	//OutString("Welcome to Nucleo F401RE\r\n");

	Delay(0xff);//ごみ値検証用

	OutString("Connected to Nucleo F401RE.");

	//クロック周波数の確認
	char str[150];
	RCC_ClocksTypeDef RCC_ClockFreq;
	RCC_GetClocksFreq(&RCC_ClockFreq);
	//printf("Frequency\r\nSYSCLK:%d\r\nHCLK:%d\r\nPCLK1:%d\r\nPCLK2:%d\r\n", RCC_ClockFreq.SYSCLK_Frequency, RCC_ClockFreq.HCLK_Frequency, RCC_ClockFreq.PCLK1_Frequency, RCC_ClockFreq.PCLK2_Frequency);
	OutString("SYSCLK:");
	itoa(str, RCC_ClockFreq.SYSCLK_Frequency);
	OutString(str); //OutString("\n\r");
	OutString("HCLK:");
	itoa(str, RCC_ClockFreq.HCLK_Frequency);
	OutString(str); //OutString("\n\r");

	
	while (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 1){
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
		Delay(0xfffff);
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
		Delay(0xfffff);
	}
	OutString("Begin EchoLoop().");
	EchoLoop();
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


