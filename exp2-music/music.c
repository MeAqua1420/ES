#include "stm32f10x.h"

//这是休止符
#define XZ 0

//C大调音阶频率
#define DO 262
#define RE 294
#define MI 330
#define FA 349
#define SO 392
#define LA 440
#define XI 494

volatile uint16_t music_tick=0;

// 大家来找茬：①下面gen_tone()函数存在3处错误，请依次找出来吧
// ②请修正下面的gen_tone()函数，使开发板能够发出正确的声音（可调用DoReMi()来验证） ^_^
void gen_tone(unsigned long freq, unsigned char time)	// freq = 发音频率, 1 time = 100 ms
{
	if(freq)
	{
		TIM1->PSC = (45000000UL/freq);	// 72M(CK_CNT)/16(TIM1_ARR) = 4.5MHz
		TIM_Cmd(TIM1, ENABLE);
	}

	while(time--)
	{
		music_tick = 10;	//发声时间(ms)
		while(music_tick);
	}
	
	TIM_Cmd(TIM1, DISABLE);

	music_tick = 90;	//间隔时间(ms)
	while(music_tick);
}

void XingXing()
{
	gen_tone(DO,5);
	gen_tone(DO,5);
	gen_tone(SO,5);
	gen_tone(SO,5);
	gen_tone(LA,5);
	gen_tone(LA,5);
	gen_tone(SO,10);

	gen_tone(FA,5);
	gen_tone(FA,5);
	gen_tone(MI,5);
	gen_tone(MI,5);
	gen_tone(RE,5);
	gen_tone(RE,5);
	gen_tone(DO,10);
	

	gen_tone(SO,5);
	gen_tone(SO,5);
	gen_tone(FA,5);
	gen_tone(FA,5);
	gen_tone(MI,5);
	gen_tone(MI,5);
	gen_tone(RE,10);
	
	gen_tone(SO,5);
	gen_tone(SO,5);
	gen_tone(FA,5);
	gen_tone(FA,5);
	gen_tone(MI,5);
	gen_tone(MI,5);
	gen_tone(RE,10);
	

	gen_tone(DO,5);
	gen_tone(DO,5);
	gen_tone(SO,5);
	gen_tone(SO,5);
	gen_tone(LA,5);
	gen_tone(LA,5);
	gen_tone(SO,10);

	gen_tone(FA,5);
	gen_tone(FA,5);
	gen_tone(MI,5);
	gen_tone(MI,5);
	gen_tone(RE,5);
	gen_tone(RE,5);
	gen_tone(DO,20);	
}

void PingGuo()
{
	gen_tone(MI*2,4);
	gen_tone(DO*2,4);
	gen_tone(RE*2,4);
	gen_tone(LA  ,4);

	gen_tone(MI*2,2);
	gen_tone(RE*2,2);
	gen_tone(DO*2,2);
	gen_tone(RE*2,2);
	gen_tone(LA  ,8);

	gen_tone(MI*2,4);
	gen_tone(DO*2,4);
	gen_tone(RE*2,4);
	gen_tone(RE*2,2);
	gen_tone(RE*2,2);

	gen_tone(SO*2,2);
	gen_tone(MI*2,2);
	gen_tone(XI  ,4);
	gen_tone(DO*2,4);

	gen_tone(DO*2,2);
	gen_tone(XI  ,2);
	gen_tone(LA  ,4);
	gen_tone(XI  ,2);
	gen_tone(DO*2,2);
	gen_tone(RE*2,4);
	gen_tone(SO  ,4);

	gen_tone(LA*2,2);
	gen_tone(SO*2,2);
	gen_tone(MI*2,4);
	gen_tone(MI*2,6);
	gen_tone(RE*2,2);

	gen_tone(DO*2,4);
	gen_tone(RE*2,2);
	gen_tone(MI*2,2);
	gen_tone(RE*2,4);
	gen_tone(SO  ,4);
	gen_tone(LA  ,4);
	gen_tone(LA  ,2);
	gen_tone(DO*2,2);
	gen_tone(LA  ,8);
}

void MoLi()
{
	gen_tone(MI*1,2);
	gen_tone(RE*1,2);
	gen_tone(MI*1,2);
	gen_tone(SO*1,2);

	gen_tone(LA*1,2);
	gen_tone(SO*1,2);
	gen_tone(DO*2,2);
	gen_tone(LA*1,2);

	gen_tone(SO*1,2);
	gen_tone(MI*1,2);
	gen_tone(SO*1,8);
	gen_tone(LA*1,4);

	gen_tone(DO*2,4);
	gen_tone(RE*2,2);
	gen_tone(MI*2,2);

	gen_tone(RE*2,2);
	gen_tone(DO*2,2);
	gen_tone(LA*1,2);
	gen_tone(DO*2,2);

	gen_tone(SO*1,16);
	
	gen_tone(SO*1,2);
	gen_tone(MI*1,2);
	gen_tone(SO*1,8);
	gen_tone(LA*1,4);

	gen_tone(DO*2,4);
	gen_tone(RE*2,2);
	gen_tone(MI*2,2);
	gen_tone(DO*2,2);
	gen_tone(LA*1,2);
	gen_tone(SO*1,4);

	gen_tone(SO*1,4);
	gen_tone(RE*1,4);
	gen_tone(MI*1,2);
	gen_tone(SO*1,2);
	gen_tone(MI*1,2);
	gen_tone(RE*1,2);

	gen_tone(DO*1,2);
	gen_tone(LA/2,2);
	gen_tone(DO*1,12);
	
	gen_tone(MI*1,2);
	gen_tone(RE*1,2);
	gen_tone(DO*1,4);
	gen_tone(RE*1,6);
	gen_tone(MI*1,2);

	gen_tone(SO*1,4);
	gen_tone(LA*1,2);
	gen_tone(DO*2,2);
	gen_tone(LA*1,4);
	gen_tone(SO*1,4);

	gen_tone(SO*1,2);
	gen_tone(MI*1,2);
	gen_tone(RE*1,4);
	gen_tone(MI*1,2);
	gen_tone(SO*1,2);
	gen_tone(MI*1,2);
	gen_tone(RE*1,2);

	gen_tone(DO*1,2);
	gen_tone(RE*1,2);
	gen_tone(LA/2,8);
	gen_tone(DO*1,4);

	gen_tone(RE*1,6);
	gen_tone(MI*1,2);
	gen_tone(DO*1,2);
	gen_tone(RE*1,2);
	gen_tone(DO*1,2);
	gen_tone(LA/2,2);

	gen_tone(DO*1,2);
	gen_tone(LA/2,2);
	gen_tone(SO/2,12);
	
}

void DoReMi()
{
	gen_tone(DO/2,1);
	gen_tone(RE/2,1);
	gen_tone(MI/2,1);
	gen_tone(FA/2,1);
	gen_tone(SO/2,1);
	gen_tone(LA/2,1);
	gen_tone(XI/2,1);
	
	gen_tone(DO*1,1);
	gen_tone(RE*1,1);
	gen_tone(MI*1,1);
	gen_tone(FA*1,1);
	gen_tone(SO*1,1);
	gen_tone(LA*1,1);
	gen_tone(XI*1,1);
	
	gen_tone(DO*2,1);
	gen_tone(RE*2,1);
	gen_tone(MI*2,1);
	gen_tone(FA*2,1);
	gen_tone(SO*2,1);
	gen_tone(LA*2,1);
	gen_tone(XI*2,1);

	gen_tone(XZ  ,10);
}

void DingDong1()
{
	gen_tone(XI*2,4);
	gen_tone(MI*2,8);
	gen_tone(XZ  ,4);
	gen_tone(XI*2,4);
	gen_tone(MI*2,8);
	gen_tone(XZ  ,4);
	gen_tone(XI*2,4);
	gen_tone(MI*2,8);
}

void DingDong2()
{
	gen_tone(DO*3,1);
	gen_tone(XI*2,1);
	gen_tone(LA*2,1);
	gen_tone(SO*2,1);
	gen_tone(MI*2,8);
	gen_tone(XZ  ,4);
	gen_tone(XI*2,1);
	gen_tone(LA*2,1);
	gen_tone(SO*2,1);
	gen_tone(FA*2,1);
	gen_tone(MI*2,8);
	gen_tone(XZ  ,4);
	gen_tone(XI*2,1);
	gen_tone(LA*2,1);
	gen_tone(SO*2,1);
	gen_tone(FA*2,1);
	gen_tone(MI*2,8);
}

