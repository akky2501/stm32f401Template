//nucleof401 stm32f401

#include "stm32f4xx.h"


//�������當����ւ̕ϊ�
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



//�O���[�o���ȃR���X�g���N�^�������ŌĂ�
void initialize(void){
	extern void(*__preinit_array_start[]) (void)__attribute__((weak));
	extern void(*__preinit_array_end[]) (void)__attribute__((weak));
	extern void(*__init_array_start[]) (void)__attribute__((weak));
	extern void(*__init_array_end[]) (void)__attribute__((weak));
	// This is basically libc_init_array -- handles global constructors

	for (int i = 0; i < __preinit_array_end - __preinit_array_start; i++)
		__preinit_array_start[i]();

	for (int i = 0; i < __init_array_end - __init_array_start; i++)
		__init_array_start[i]();
}

namespace GPIO{
	class AssignmentHolder{
		//		friend int16_t operator=(int16_t& v, AssignmentHolder& ah);
		//		friend int16_t operator=(AssignmentHolder& ah, int16_t&);
	public:
		AssignmentHolder(GPIO_TypeDef* gpio, uint16_t pin);//GPIO_Pin_ALL is not avairable.
		GPIO_TypeDef* getGPIO(void);
		uint16_t getPin(void);
		uint16_t getPinSource(void);
	private:
		AssignmentHolder();
		//AssignmentHolder(const AssignmentHolder&){gpio_ = nullptr; pin_ = 0;  }
		//const AssignmentHolder& operator=(const AssignmentHolder&){ return *this; }

		GPIO_TypeDef* gpio_;
		uint16_t pin_;
	};

	AssignmentHolder::AssignmentHolder(GPIO_TypeDef* gpio, uint16_t pin)
		:gpio_(gpio), pin_(pin)
	{}

	GPIO_TypeDef* AssignmentHolder::getGPIO(void){
		return gpio_;
	}

	uint16_t AssignmentHolder::getPin(void){
		return pin_;
	}

	uint16_t AssignmentHolder::getPinSource(void){
		uint16_t mask = 0x01;
		uint16_t i = 0;
		while (!(pin_ & mask)){
			i++;
			mask <<= 1;
		}
		return i;
	}

}

namespace IO{

	//nucleo interface
	GPIO::AssignmentHolder user_led(GPIOA, GPIO_Pin_5);//CN10.11
	GPIO::AssignmentHolder user_switch(GPIOC, GPIO_Pin_13);//CN7.23


	void setting(void){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

		GPIO_InitTypeDef GPIO_InitStructure = {};

		//nucleo LED
		GPIO_InitStructure.GPIO_Pin = user_led.getPin();
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(user_led.getGPIO(), &GPIO_InitStructure);


		//nuclleo Pushswitch
		GPIO_InitStructure.GPIO_Pin = user_switch.getPin();//�����̓n�[�h�I��PULLUP����Ă�
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(user_switch.getGPIO(), &GPIO_InitStructure);

	}

	//�������ݕ⏕
	//�ϒ������ւ̑Ή�->�|�[�g�Ⴄ�Ƃ��߂�����ۗ�
	void Set(GPIO::AssignmentHolder& ah){
		GPIO_SetBits(ah.getGPIO(), ah.getPin());
	}

	void Reset(GPIO::AssignmentHolder& ah){
		GPIO_ResetBits(ah.getGPIO(), ah.getPin());
	}

	void Write(GPIO::AssignmentHolder& ah, int16_t v){
		GPIO_WriteBit(ah.getGPIO(), ah.getPin(), v ? Bit_SET : Bit_RESET);
	}
	/*
	int operator=(AssignmentHolder& ah, int v){
	Write(ah, v);
	return v;
	}
	*/
	//�ǂݍ��ݕ⏕
	int16_t Read(GPIO::AssignmentHolder& ah){
		return (int16_t)GPIO_ReadInputDataBit(ah.getGPIO(), ah.getPin());
	}
	/*
	int16_t operator=(int16_t& v, AssignmentHolder& ah){
	v = Read(ah);
	return v;
	}

	bool operator==(int v, GPIO::AssignmentHolder& ah){
	return v == Read(ah);
	}

	bool operator==(GPIO::AssignmentHolder& ah, int v){
	return v == Read(ah);
	}*/
}

namespace USART{

	//USART2 module
	GPIO::AssignmentHolder usart_tx(GPIOA, GPIO_Pin_2);//CN10.35
	GPIO::AssignmentHolder usart_rx(GPIOA, GPIO_Pin_3);//CN10.37


	void setting(void){
		//�N���b�N����
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

		GPIO_InitTypeDef GPIO_InitStructure = {};
		GPIO_InitStructure.GPIO_Pin = usart_tx.getPin() | usart_rx.getPin();
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(usart_tx.getGPIO(), &GPIO_InitStructure);

		/* Connect USART pins to AF */
		GPIO_PinAFConfig(usart_rx.getGPIO(), usart_rx.getPinSource(), GPIO_AF_USART2);
		GPIO_PinAFConfig(usart_tx.getGPIO(), usart_tx.getPinSource(), GPIO_AF_USART2);

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

	void output(uint8_t data){
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty

		USART_SendData(USART2, data);
	}

	void outputString(char* str){
		while (*str)
		{
			while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty

			USART_SendData(USART2, *str++); // Send Char
		}
	}

	class Stream{
	public:
		Stream(){}
		~Stream(){}

		Stream& operator<<(char* s){
			outputString(s);
			return *this;
		}

		Stream& operator<<(int n){
			char str[16];
			itoa(str, n);
			outputString(str);
			return *this;
		}

		Stream& operator<<(Stream& (*manipulator)(Stream&)){
			return manipulator(*this);
		}
	};

	Stream& endl(Stream& st){
		outputString("\n\r");
		return st;
	}


	Stream out;
}

namespace SPI{

	//SPI1 module for moter driving
	GPIO::AssignmentHolder spi1_clk(GPIOB, GPIO_Pin_3);//CN10.31
	GPIO::AssignmentHolder spi1_miso(GPIOB, GPIO_Pin_4);//CN10.27
	GPIO::AssignmentHolder spi1_mosi(GPIOB, GPIO_Pin_5);//CN10.29
	//SPI2 module for moter driving
	GPIO::AssignmentHolder spi2_clk(GPIOB, GPIO_Pin_13);//CN10.30
	GPIO::AssignmentHolder spi2_miso(GPIOB, GPIO_Pin_14);//CN10.28
	GPIO::AssignmentHolder spi2_mosi(GPIOB, GPIO_Pin_15);//CN10.26


	void setting(void){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

		GPIO_InitTypeDef GPIO_InitStructure = {};

		GPIO_InitStructure.GPIO_Pin = spi1_clk.getPin() | spi1_miso.getPin() | spi1_mosi.getPin()
									| spi2_clk.getPin() | spi2_miso.getPin() | spi2_mosi.getPin();
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(spi1_clk.getGPIO(), &GPIO_InitStructure);

		GPIO_PinAFConfig(spi1_clk.getGPIO(),  spi1_clk.getPinSource(),  GPIO_AF_SPI1);
		GPIO_PinAFConfig(spi1_miso.getGPIO(), spi1_miso.getPinSource(), GPIO_AF_SPI1);
		GPIO_PinAFConfig(spi1_mosi.getGPIO(), spi1_mosi.getPinSource(), GPIO_AF_SPI1);

		GPIO_PinAFConfig(spi2_clk.getGPIO(),  spi2_clk.getPinSource(),  GPIO_AF_SPI2);
		GPIO_PinAFConfig(spi2_miso.getGPIO(), spi2_miso.getPinSource(), GPIO_AF_SPI2);
		GPIO_PinAFConfig(spi2_mosi.getGPIO(), spi2_mosi.getPinSource(), GPIO_AF_SPI2);



		//SPI setting
		/* SPI1 and SPI2 configuration */
		SPI_InitTypeDef SPI_InitStructure = { 0 };
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//�ʏ펞��High,Low
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//���b�`
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		
		SPI_Init(SPI1, &SPI_InitStructure);
		SPI_Init(SPI2, &SPI_InitStructure);
		
		SPI_Cmd(SPI1, ENABLE);
		SPI_Cmd(SPI2, ENABLE);
	}

	uint8_t transaction1(uint8_t data){
		uint16_t RxData;
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);		// wait until transmit complete
		SPI_I2S_SendData(SPI1, data);										// write data to be transmitted to the SPI data register
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);	// wait until receive complete
		RxData = SPI_I2S_ReceiveData(SPI1);								// return received data from SPI data register
		return (uint8_t)RxData;
	}

	uint8_t transaction2(uint8_t data)
	{
		uint16_t RxData;
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);		// wait until transmit complete
		SPI_I2S_SendData(SPI2, data);										// write data to be transmitted to the SPI data register
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);	// wait until receive complete
		RxData = SPI_I2S_ReceiveData(SPI2);								// return received data from SPI data register
		return (uint8_t)RxData;
	}


}

namespace Timer{
	void setting(void){
		//�N���b�N����
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //�^�C�}2�L����

		//���荞�ݗD�揇�ʐݒ�
		NVIC_InitTypeDef NVIC_InitStructure = { 0 };
		//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
		// Enable the TIM2 gloabal Interrupt 
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		//	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

		//�^�C�}�ݒ�
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = { 0 };
		// 42MHz*2=84MHz? -> 10kHz -> 10kHz -> 1Hz
		TIM_TimeBaseStructure.TIM_Period = 100 - 1;//���J�E���g�����犄�荞�݂��邩10kHz-->100Hz ==> PID��������?
		TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;//��J�E���g����̂ɉ��N���b�N�v�����邩84MHz-->10kHz
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//84MHz / 1 = 84MHz
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	}

	void begin2(void){
		TIM_Cmd(TIM2, ENABLE);
	}
}

namespace ADConverter{

	//AD�|�[�g�̂Ƃ��낾��
	GPIO::AssignmentHolder adpin[] = {
		GPIO::AssignmentHolder(GPIOA, GPIO_Pin_0),//CN7.28,ADC1_IN0
		GPIO::AssignmentHolder(GPIOA, GPIO_Pin_1),//CN7.30,ADC1_IN1
		GPIO::AssignmentHolder(GPIOA, GPIO_Pin_4),//CN7.32,ADC1_IN4
		GPIO::AssignmentHolder(GPIOB, GPIO_Pin_0),//CN7.34,ADC1_IN8
		GPIO::AssignmentHolder(GPIOC, GPIO_Pin_1),//CN7.36,ADC1_IN11
		GPIO::AssignmentHolder(GPIOC, GPIO_Pin_0) //CN7.38,ADC1_IN10
	};

	void setting(void){
		//�N���b�N�̋���
		ADC_DeInit();
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

		//ADC�̃L�����u���[�V����(f4�͂���Ȃ����ۂ�)

		//AD�ϊ��p(GPIOA�̎w��̓��͂��A�i���O�ɐݒ�)
		GPIO_InitTypeDef ADC_GPIO_InitStructure = { 0 };

		ADC_GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		ADC_GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		ADC_GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		ADC_GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		for (auto& p : adpin){
			ADC_GPIO_InitStructure.GPIO_Pin = p.getPin();	//Pin��ݒ�
			GPIO_Init(p.getGPIO(), &ADC_GPIO_InitStructure);
		}

		//ADC�̐ݒ�
		ADC_CommonInitTypeDef ADC_CommonInitStructure = { 0 };
		ADC_InitTypeDef ADC_InitStructure = { 0 };

		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
		ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
		ADC_CommonInit(&ADC_CommonInitStructure);

		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfConversion = 6;//�g���`�����l����
		ADC_Init(ADC1, &ADC_InitStructure);

		
		ADC_Cmd(ADC1, ENABLE);
	}

	//12bit A/D converter
	#define getch(x)	\
	int16_t getch##x(void){\
		ADC_RegularChannelConfig(ADC1, ADC_Channel_##x, 1, ADC_SampleTime_3Cycles);\
		ADC_SoftwareStartConv(ADC1);\
		while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);\
		return ADC_GetConversionValue(ADC1);\
	}

	//1ch���Ƃ�ADC�֐�
	getch(0)
	getch(1)
	getch(4)
	getch(8)
	getch(11)
	getch(10)

	int16_t(*getch_array[6])(void) = { getch0, getch1, getch4, getch8, getch11, getch10 };

	//DMA�⊄�荞�݂��g�킸�Ɏ擾
	int16_t get(unsigned char ch){
			return ch <= 5 ? getch_array[ch]() : -1;
	}


}


//SysTick���g�����I
void Delay(__IO uint32_t nCount) {
	for (; nCount != 0; nCount--)
		;
}


//�p�����[�^
const char motor_duty_max = 125;//���[�^�̍ō����x
const char motor_duty_min = -125;//���[�^�̍Œᑬ�x

const char motorL_duty_initial = 60;//���[�^�̏������x(��)
const char motorR_duty_initial = 60;//(�E)

/*
const char motorL_duty_go = 60;//���i���x
const char motorR_duty_go = 60;

const char motorL_duty_turnL = -60;//������
const char motorR_duty_turnL = 60;

const char motorL_duty_turnR = 60;//�E����
const char motorR_duty_turnR = -60;

const char motorL_duty_back = -60;//��i
const char motorR_duty_back = -60;

const char motorL_duty_roundL = -60;//������
const char motorR_duty_roundL = 60;

const char motorL_duty_roundR = 60;//������
const char motorR_duty_roundR = -60;
*/

const int line_white_threshold = 0;//�Z���T�����C����ɂ��邩�ǂ�����臒l
const int line_target[5] = {0,0,0,0,0};	//���[�^��P(ID)���䂷��ۂ̃Z���T�[�ڕW�l
const float Kp = 0;//PID���p�����[�^
//const float Ki = 0;
//const float Kd = 0;

//�ϐ�
char motor_error_flag = 0;	//���[�^�֌W�̃G���[�R�[�h
bool do_linetrace = false;	//���C���g���[�X���邩�̃t���O
int line_countL = 0; //�����̃J�E���g(��)
int line_countR = 0;//(�E)
int line_countV = 0;//(�c)

char motorL_duty = motorL_duty_initial;	//�����郂�[�^�[���x(��)
char motorR_duty = motorR_duty_initial;	//(�E)


template<typename T>
T clamp(T x, T mint, T maxt){
	return x >= mint ? (x <= maxt ? x : maxt) : mint;
}

char clamp_duty(char duty){
	return clamp(duty, motor_duty_min, motor_duty_min);
}


void hold(void){
	//�r�����
	//�T�[�{��duty�ύX
}

void release(void){
	//�r���J������
	//�T�[�{��duty�ύX
}

void go(int k){
	//k���(k�{�̐������؂�)���C���g���[�X����
	//���C���g���[�X���Ȃ�����𐔂���

	//���C���g���[�X���[�h�ɂ��āA���������̖{���𐔂���
	do_linetrace = true;
	//
	do_lintrace = false;
}

void back(int k){
	//k���(k�{�̐������؂�)�o�b�N����
	//�o�b�N���Ȃ�����𐔂���
}

void turnRoundL(void){
	//180�x��]����(�c����2or3�{���؂�?)
}

void turnRoundR(void){
	//180�x��]����(�c����2or3�{���؂�?)
}

void turnL(void){
	//�\���H�ō���90�x���(�^�񒆃Z���T�[����������܂ŉ��)
}

void turnR(void){
	//�\���H�ŉE��90�x���(�^�񒆃Z���T�[����������܂ŉ��)
}



int main(void)
{
	initialize();

	SystemInit();

	IO::setting();
	USART::setting();
	SPI::setting();
	//ADConverter::setting();

	Timer::setting();

	Delay(0xff);

	
	USART::out << "Connected to Nucleo F401RE." << USART::endl;

	//�N���b�N���g���̊m�F
	RCC_ClocksTypeDef RCC_ClockFreq;
	RCC_GetClocksFreq(&RCC_ClockFreq);
	//printf("Frequency\r\nSYSCLK:%d\r\nHCLK:%d\r\nPCLK1:%d\r\nPCLK2:%d\r\n", RCC_ClockFreq.SYSCLK_Frequency, RCC_ClockFreq.HCLK_Frequency, RCC_ClockFreq.PCLK1_Frequency, RCC_ClockFreq.PCLK2_Frequency);
	USART::out << "SYSCLK:" << RCC_ClockFreq.SYSCLK_Frequency << USART::endl
		<< "HCLK:" << RCC_ClockFreq.HCLK_Frequency << USART::endl;

	//���[�^SPI������
	if ((char)SPI::transaction1(0) == -128) USART::out << "get initial value L\n\r";
	for (int i = 0; i < 20; i++) SPI::transaction1(0);
	if ((char)SPI::transaction2(0) == -128) USART::out << "get initial value R\n\r";
	for (int i = 0; i < 20; i++) SPI::transaction2(0);

	//���샂�[�h�I��
	while (IO::Read(IO::user_switch));

	

	Delay(0xff);

	Timer::begin2();//���[�^�p���荞�ݔ����J�n

	while (1){
		/*
		if (IO::Read(IO::user_switch)){	//pull
			IO::Reset(IO::user_led);//on
		}
		else{	//push
			IO::Set(IO::user_led);//off
		}
		*/
		if (motor_error_flag != 0){
			USART::out << "moter_error:" << motor_error_flag << USART::endl;
			IO::Reset(IO::user_led);//on

			//motor_error_flag = 0;
		}
		

	}

	USART::out << "Exit!";//���肦�Ȃ�
}

void blink(void){
	// ����͊ȒP�Ȃ̂Œ��ɏ����B��������֐��փW�����v����̂�����B

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



void motor_control(void){

	static char prev_dutyL = 0;//�G���[���o�p
	static char prev_dutyR = 0;//�G���[���o�p

	char dutyL = motorL_duty;
	char dutyR = motorR_duty;

	//int a = ADConverter::get(0);
	//int b = ADConverter::get(1);

	//���x����A���C���g���[�X
	if (do_linetrace){
		motorL_duty = ;
	}
	//duty_motorL = a>2000?50:0;
	//duty_motorR = b>2000?50:0;
	//duty_motorL++;
	//duty_motorR++;


	//�f�[�^�]��
	char datL = (char)SPI::transaction1((uint8_t)dutyL);
	if (datL != prev_dutyL){
		motor_error_flag = 1;
	}

	char datR = (char)SPI::transaction2((uint8_t)dutyR);
	if (datR != prev_dutyR){
		motor_error_flag = 2;
	}

	//�ߋ��l�̍X�V
	prev_dutyL = dutyL;
	prev_dutyR = dutyR;
	
}


//Interrupt Function for TIM2
//���[�^�[�̑��x����
extern "C" void TIM2_IRQHandler(void)
{
	// �O���[�o�����荞�݂���ʗv���փW�����v���邽�߂Ƀt���O�`�F�b�N����
	// ����͗v����1�����Ȃ����ǁA�ʏ�͂��������X�^�C���ŕ��򂷂�
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		// ���荞�ݕۗ��r�b�g(=���荞�ݗv���t���O)���N���A
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		motor_control();
	}
}

/**************************************************************************************/

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
	while (1)
	{
	}
}
#endif

static __IO uint32_t uwTimingDelay;
void TimingDelay_Decrement(void)
{
	if (uwTimingDelay != 0x00)
	{
		uwTimingDelay--;
	}
}


