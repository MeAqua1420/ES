#include "stm32f10x.h"
#include "stm32f10x_tim.h"

const uint32_t fonts[]={0x00c0003f,0x00f90006,0x00a4005b,0x00b0004f,
												0x00990066,0x0092006d,0x0082007d,0x00d80027,
												0x0080007f,0x0090006f,0x00880077,0x0083007c,
												0x00c60039,0x00a1005e,0x00860079,0x008e0071};

volatile uint32_t disp[]={0x00ff0000,0x00890076,0x00cf0030,0x00ff0000};

volatile uint8_t tick=0;
volatile uint8_t id=0;

volatile uint16_t music_tick=0;

extern const unsigned char dingdong1[];
extern const unsigned char dingdong2[];
extern const unsigned char audio2[];	// 这是一段采样率为6kHz的音频片段，长度为55872个采样，请尝试正确的播放它 ^_^

volatile const uint8_t *AudioData = dingdong1;
volatile uint16_t AudioDataLen = 9104;

USART_InitTypeDef USART_InitStructure;

void clock_init()
{
  /* GPIOA and GPIOB and TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_USART2,	ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1 |
												 RCC_APB2Periph_USART1,	ENABLE);
}

void pin_remap_init()
{
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);	
}

void gpio_init()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_15;	// PA15 - TIM
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;	// PB10,PB11 - TIM
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_9;	// PA2,PA9 - USART2,USART1
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_10;	// PA3,PA10 - USART2,USART1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = 0xf0ff;	// PB0-7,PB12-15 - LED_7SEG
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void timer_init()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	uint16_t CCR1_Val = 0;
	uint16_t CCR3_Val = 0;
	uint16_t CCR4_Val = 0;
	uint16_t PrescalerValue = 0;
	
	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 59999;	// 计数周期=1000
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

  TIM_OC3Init(TIM2, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

  TIM_OC4Init(TIM2, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM2, ENABLE);

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
}

void audio_init()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	uint16_t CCR1_Val = 1;
	uint16_t PrescalerValue = 0;
	
	/* Compute the prescaler value */
  //PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	PrescalerValue = 0;
	
  /* Time base configuration */	// 计数时钟=24MHz/2=12MHz
  TIM_TimeBaseStructure.TIM_Period = 255;	// 播放8bit波形数据
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_CCPreloadControl(TIM1, DISABLE);	// 关闭捕获比较通道预加载（为互补PWM输出同步更新正负通道准备的，而我们并非互补PWM输出，所以禁止）
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	
  /* TIM1 enable counter */
  TIM_Cmd(TIM1, ENABLE);
}

void usart_init()
{
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART1, &USART_InitStructure);
  USART_Init(USART2, &USART_InitStructure);
	
	USART_Cmd(USART1, ENABLE);
	USART_Cmd(USART2, ENABLE);
}

void usart_putchar(USART_TypeDef* USART, char ch)
{
	// 通过USART向外发送数据
  USART_SendData(USART, ch);
	// 等待发送缓冲区变成空的（就是发完了的意思）
	while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET){}
}

void usart_putstr(USART_TypeDef* USART, char *s)
{
	while(*s)
	{
		usart_putchar(USART, *s++);
	}
}

char usart_recv(USART_TypeDef* USART)
{
	// 接收缓冲区非空（就是收到了数据的意思）
	if(USART_GetFlagStatus(USART, USART_FLAG_RXNE) == SET)
	{
		return USART_ReceiveData(USART);
	}
	else
	{
		return 0;
	}
}

int main(void)
{
	clock_init();
	
	pin_remap_init();

	gpio_init();
	
	timer_init();
	
	audio_init();
	
	usart_init();
	
	SysTick_Config(SystemCoreClock / 8000);	// 中断频率8000Hz
	
	usart_putstr(USART1,"Hello world !\r\n");
	
	disp[0]=0x00ff0000;
	disp[1]=fonts[2];
	disp[2]=0x00bf0040;
	disp[3]=fonts[2];
	
  while (1)
  {
		char c;
		
		if(tick)
		{
			id-=3;
			id&=0xf;
			tick=0;
		}
		
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12) == Bit_RESET)
		{
			AudioData = dingdong1;
			AudioDataLen = 9104;
			music_tick=0;
			usart_putstr(USART1," > KEY1 PRESS\r\n");
			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12) == Bit_RESET);
			usart_putchar(USART2,'A');
		}
		
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11) == Bit_RESET)
		{
			id=0;
			AudioData = dingdong2;
			AudioDataLen = 9600;
			music_tick=0;
			usart_putstr(USART1," > KEY2 PRESS\r\n");
			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11) == Bit_RESET);
			usart_putchar(USART2,'B');
		}
		
		c = usart_recv(USART1);
		
		if(c != 0)
		{
			usart_putstr(USART1," >> USART1 INPUT: ");
			usart_putchar(USART1,c);			
			usart_putstr(USART1,"\r\n");			
		}

		c = usart_recv(USART2);
		
		if(c != 0)
		{
			usart_putstr(USART1," >> USART2 INPUT: ");
			usart_putchar(USART1,c);			
			usart_putstr(USART1,"\r\n");			
		}
  }
}
