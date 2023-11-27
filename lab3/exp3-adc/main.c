#include "stm32f10x.h"
#include "stm32f10x_tim.h"

const uint32_t fonts[]={0x00c0003f,0x00f90006,0x00a4005b,0x00b0004f,
												0x00990066,0x0092006d,0x0082007d,0x00d80027,
												0x0080007f,0x0090006f,0x00880077,0x0083007c,
												0x00c60039,0x00a1005e,0x00860079,0x008e0071};

volatile uint32_t disp[]={0x00ff0000,0x00890076,0x00cf0030,0x00ff0000};

volatile uint16_t adc_value[2];	// X Y value

volatile uint8_t dly=0;

void clock_init()
{
  /* GPIOA and GPIOB and TIM2 clock enable */
	RCC_ADCCLKConfig(RCC_PCLK2_Div4);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_USART2,	ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1 |
												 RCC_APB2Periph_USART1 | RCC_APB2Periph_ADC1,	ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
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
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;	// PA0,PA1 - ADC
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
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
	USART_InitTypeDef USART_InitStructure;
	
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

uint8_t ReadKey1()
{
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12) == Bit_RESET)
	{
		usart_putstr(USART1," > KEY1 PRESS\r\n");
		while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12) == Bit_RESET);
		usart_putstr(USART1,"   ==> KEY1 RELEASE\r\n");
		return 1;
	}
	return 0;
}

uint8_t ReadKey2()
{
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11) == Bit_RESET)
	{
		usart_putstr(USART1," > KEY2 PRESS\r\n");
		while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11) == Bit_RESET);
		usart_putstr(USART1,"   ==> KEY2 RELEASE\r\n");
		return 1;
	}
	return 0;
}

void adc_dma_init()
{
	#define ADC1_DR_Address    ((uint32_t)0x4001244C)
	
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
  /* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adc_value[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 2;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel14 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void print_uint16(uint16_t dat)
{
	char buf[6];	// 发送缓冲区
	
	buf[0]='0'+(dat/10000);
	dat%=10000;
	buf[1]='0'+(dat/1000);
	dat%=1000;
	buf[2]='0'+(dat/100);
	dat%=100;
	buf[3]='0'+(dat/10);
	dat%=10;
	buf[4]='0'+(dat/1);
	buf[5]=0;
	
	usart_putstr(USART1,buf);
}

int main(void)
{
	// 操作流程：
	// ①编译程序并烧写到开发板上
	// ②点击工具栏上的【放大镜d】图标，进入调试模式
	// ③将摇杆拨动到不同角度，观察adc_value[]数组的值的变化
	// ④动手加上液晶屏显示功能，将adc_value[]的值实时呈现在屏幕上
	
	clock_init();
	
	pin_remap_init();

	gpio_init();
	
	timer_init();
	
	audio_init();
	
	usart_init();
	
	adc_dma_init();
	
	SysTick_Config(SystemCoreClock / 1000);	// 中断频率1000Hz
	
	
	while(1)
	{
		usart_putstr(USART1,"ADC Value: X=");
		print_uint16(adc_value[0]);
		usart_putstr(USART1,", Y=");
		print_uint16(adc_value[1]);
		usart_putstr(USART1,", \r\n");
	}
}
