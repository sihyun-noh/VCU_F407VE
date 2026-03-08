/*该功能为将PB0 PA4 PA6的模拟输入量转化为数字量 由DMA循环转运节省CPU资源，并由串口1将数据打印到终端*/
/*ad循环扫描 dma循环搬运 由串口1将数据打印到终端*/
/*该功能为将PB0 PA4 PA6的模拟输入量转化为数字量 由DMA转运节省CPU资源*/

#include "AD.h"
u16 AD_Value[5] = {0};


void Init_AD(void)
{
	//使能ADC1和GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	//初始化GPIOA配置为模拟输入
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;												//配置为模拟输入
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;												//配置为
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;													//配置为pB0口
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;											//配置为浮空输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	

	//初始化AD结构体
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_ScanConvMode =ENABLE;													//配置为开启扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;											//使能循环采集
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;										//设置右对齐
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;						//使用软件触发，不使用硬件触发，该参数随意配置
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;					//配置为不启用硬件触发，也就是软件触发
	ADC_InitStructure.ADC_NbrOfConversion = 5;													//ad转换的通道数量，也就是配置规则组里面通道的个数 这里配置为3个
	ADC_InitStructure.ADC_Resolution = ADC_TwoSamplingDelay_8Cycles;							//配置分辨率
	ADC_Init(ADC1, &ADC_InitStructure);

	// -------------------ADC Common 结构体 参数 初始化---------------------
	
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;						//禁止DMA直接访问，只有在双重或者三重模式才需要设置
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;									//有独立模式，双重模式和三重模式，这里为独立模式采集，一般用独立模式
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;									//ADC时钟分频系数选择 最大不分频为36MHZ 这里为6分频
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;				//采样时间间隔

	ADC_CommonInit(&ADC_CommonInitStructure);

	/*ADC共有16个复用通道*/
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10, 1, ADC_SampleTime_15Cycles);					//将ADC通道0中的数据放入规则组第一个空间进行模数转换
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_12, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_13, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_14, 5, ADC_SampleTime_15Cycles);

	//------------------------------DMA初始化-----------------------------------
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;								//寄存器地址为adc1的数据寄存器地址
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;							//单次模式  AD采集要求是单次模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;					//外设数据大小，半字
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;							//指定外设地址不自增
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)AD_Value;									//存储器地址为栈空间里的一块区域，相当于一个变量的地址
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;									//存储器突发模式选择，ADC 采集传输是直接模式，要求使用单次模式
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;							//设置存储器大小为半字
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;										//设置存储器地址自增
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;												//设置dma循环搬运还是一次搬运 我们设置循环搬运										
	DMA_InitStructure.DMA_BufferSize = 5;														//设定待传输数据的数目，我们这里一轮转换5个所以是5
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;										//指定搬运方向，我们这里是外设到存储器
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;										//ADC 采集传输使用直接传输模式即可，不需要使用FIFO
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;						//DMA_FIFOMode 设置为 DMA_FIFOMode_Disable，那 DMA_FIFOThreshold 值无效。ADC采集传输不使用 FIFO 模式，设置该值无效。
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;											//设置优先级为高
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;												//指定要搬运的DMA的通道，不同的外设通道不一样
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);													//初始化DMA流，流相当于大河，河中放了很多通道
	
	DMA_Cmd(DMA2_Stream0, ENABLE);																//给DMA上电

	ADC_DMARequestAfterLastTransferCmd(ADC1,ENABLE);											//使能DMA请求 after last transfer (Single-ADC mode)
	
	ADC_DMACmd(ADC1, ENABLE);																	//ADC1启用DMA数据搬运
	
	ADC_Cmd(ADC1, ENABLE);																		//给adc上电
	
	ADC_SoftwareStartConv(ADC1);																//软件触发开始转换数据

}



//若使用DMA搬运，dma会自动将转换后的值搬运到指定位置，则不需要使用该函数
uint16_t Get_AD_Value(void)
{
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) == RESET);
	ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
	return ADC_GetConversionValue(ADC1);
}



static void battery_thread_entry(void* parameter)
{	
	//计算好的AD值放入其中
	float ad_value[5] = {0};

	while (1)
	{
		for(int i = 0;i<5;i++)
		{
			rt_kprintf("%d ",AD_Value[i]);							//没问题	
		}
		rt_kprintf("\n");
		for(int i = 0;i<5;i++)
		{
			ad_value[i] = (float)AD_Value[i]/4096*3.3;
			rt_kprintf("%.2fv ",ad_value[i]);							//没问题
		}
		rt_kprintf("\n");
	rt_thread_delay(500);   /* 延时1000个tick */
	}
}  



/* 定义线程控制块 */
static rt_thread_t battery_thread = RT_NULL;
int bsp_battery_thread(void)
{
	battery_thread =                          /* 线程控制块指针 */
    rt_thread_create( "battery",              /* 线程名字 */
                      battery_thread_entry,   /* 线程入口函数 */
                      RT_NULL,             /* 线程入口函数参数 */
                      512,                 /* 线程栈大小 */
                      3,                   /* 线程的优先级 */
                      20);                 /* 线程时间片 */

    /* 启动线程，开启调度 */
   if (battery_thread != RT_NULL)
        rt_thread_startup(battery_thread);
    else
        return -1;
}


