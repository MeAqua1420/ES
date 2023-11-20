#include "stm32f10x.h"
#include "stm32f10x_tim.h"

const uint32_t fonts[]={0x00c0003f,0x00f90006,0x00a4005b,0x00b0004f,
												0x00990066,0x0092006d,0x0082007d,0x00d80027,
												0x0080007f,0x0090006f,0x00880077,0x0083007c,
												0x00c60039,0x00a1005e,0x00860079,0x008e0071};

volatile uint32_t disp[]={0x00ff0000,0x00890076,0x00cf0030,0x00ff0000};

volatile uint8_t tick=0;
volatile uint8_t id=0;

void XingXing(void);
void PingGuo(void);
void MoLi(void);
void DoReMi(void);
void DingDong1(void);
void DingDong2(void);

void clock_init()
{
  /* GPIOA and GPIOB and TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,	ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1,	ENABLE);
}

void pin_remap_init()
{
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);	
}

void gpio_init()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = 0xf0ff;	// PB0-7,PB12-15
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
  TIM_TimeBaseStructure.TIM_Period = 999;	// ��������=1000
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
	
	uint16_t CCR1_Val = 8;
	uint16_t PrescalerValue = 0;
	
	/* Compute the prescaler value */
	PrescalerValue = 0;	// �ȳ�ʼ��Ϊ0��ʵ����Ƶ��������������ֵ
	
  /* Time base configuration */	// ����ʱ��=24MHz/2=12MHz
  TIM_TimeBaseStructure.TIM_Period = 15;	// ռ�ձ�8/16 = 50%
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;	// �ظ��������������˴���ʹ�ã��ɳ�ʼ��Ϊ0

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_CCPreloadControl(TIM1, DISABLE);	// �رղ���Ƚ�ͨ��Ԥ���أ�Ϊ����PWM���ͬ����������ͨ��׼���ģ������ǲ��ǻ���PWM��������Խ�ֹ�������޷���ȷ���PWM�źţ�
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);	// �߼���ʱ��TIM1��TIM8���еĹ��ܣ���Ҫ�ֹ���������ʵ��PWM�źŵ����
	
  /* TIM1 enable counter */
//  TIM_Cmd(TIM1, ENABLE);	// ��ʼ��ʱ����������ʱ���������Ƶ�����Ὺ����
}

int main(void)
{
	clock_init();
	
	pin_remap_init();

	gpio_init();
	
	timer_init();
	
	audio_init();
	
	SysTick_Config(SystemCoreClock / 1000);	// 1ms�ж�һ��
	
	DingDong1();
	//DoReMi();
	//MoLi();
	//PingGuo();
	//XingXing();

	disp[0]=0x00ff0000;
	disp[1]=fonts[2];
	disp[2]=0x00bf0040;
	disp[3]=fonts[1];
	
  while (1)
  {
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12) == Bit_RESET)	// KEY1����
		{
			PingGuo();
		}
		
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11) == Bit_RESET)	// KEY2����
		{
			XingXing();
		}
  }
}
