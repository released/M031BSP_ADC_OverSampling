/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include	"project_config.h"


/*_____ D E C L A R A T I O N S ____________________________________________*/
enum
{
	ADC0_CH0 = 0 ,
	ADC0_CH1 , 
	
	ADC0_CH2 , 
	ADC0_CH3 , 
	ADC0_CH4 , 
	ADC0_CH5 , 
	ADC0_CH6 , 
	ADC0_CH7 , 
	ADC0_CH8 , 
	ADC0_CH9 , 
	ADC0_CH10 , 
	ADC0_CH11 , 
	ADC0_CH12 ,
	ADC0_CH13 , 
	ADC0_CH14 , 
	ADC0_CH15 , 
	
	ADC_CH_DEFAULT 	
}ADC_CH_TypeDef;
/*_____ D E F I N I T I O N S ______________________________________________*/
volatile uint32_t BitFlag = 0;
volatile uint32_t counter_tick = 0;

#define ADC_RESOLUTION								((uint16_t)(4096u))
#define ADC_REF_VOLTAGE								((uint16_t)(3300u))	//(float)(3.3f)

#define ADC_MAX_TARGET								((uint16_t)(4095u))	//(float)(2.612f)
#define ADC_MIN_TARGET								((uint16_t)(0u))	//(float)(0.423f)
#define DUTY_MAX									(uint16_t)(100)
#define DUTY_MIN									(uint16_t)(0)

#define ADC_CONVERT_TARGET							(float)(ADC_MIN_TARGET*ADC_RESOLUTION/ADC_REF_VOLTAGE) //81.92000 
#define ADC_SUB_TARGET								(float)((ADC_MAX_TARGET-ADC_MIN_TARGET)/(DUTY_MAX-DUTY_MIN)*(ADC_RESOLUTION/ADC_REF_VOLTAGE))//5.60505 

#define ADC_OVER_SAMPLE_COUNT 					(uint16_t)(256)	
#define ADC_OVER_SAMPLE_SHIFT 						(uint16_t)(4)	
uint32_t oversamplingSum = 0;

uint16_t aADCxConvertedData = 0;	

//uint16_t adcRawData_Target = 0;
uint16_t movingAverage_Target = 0;
uint32_t movingAverageSum_Target = 0;

uint32_t AVdd = 0;

#define ADCextendSampling 							(0)
/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void tick_counter(void)
{
	counter_tick++;
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			set_flag(flag_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		set_flag(flag_error , DISABLE);
	}

}

void reset_buffer(void *dest, unsigned int val, unsigned int size)
{
    uint8_t *pu8Dest;
//    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;

	#if 1
	while (size-- > 0)
		*pu8Dest++ = val;
	#else
	memset(pu8Dest, val, size * (sizeof(pu8Dest[0]) ));
	#endif
	
}

void copy_buffer(void *dest, void *src, unsigned int size)
{
    uint8_t *pu8Src, *pu8Dest;
    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;


	#if 0
	  while (size--)
	    *pu8Dest++ = *pu8Src++;
	#else
    for (i = 0; i < size; i++)
        pu8Dest[i] = pu8Src[i];
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void  dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}

void delay(uint16_t dly)
{
/*
	delay(100) : 14.84 us
	delay(200) : 29.37 us
	delay(300) : 43.97 us
	delay(400) : 58.5 us	
	delay(500) : 73.13 us	
	
	delay(1500) : 0.218 ms (218 us)
	delay(2000) : 0.291 ms (291 us)	
*/

	while( dly--);
}


void delay_ms(uint16_t ms)
{
	TIMER_Delay(TIMER0, 1000*ms);
}


__STATIC_INLINE uint32_t FMC_ReadBandGap(void)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;            /* Set ISP Command Code */
    FMC->ISPADDR = 0x70u;                         /* Must keep 0x70 when read Band-Gap */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;           /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                            /* To make sure ISP/CPU be Synchronized */
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) {}  /* Waiting for ISP Done */

    return FMC->ISPDAT & 0xFFF;
}

void ADC_IRQHandler(void)
{
	
//	printf("aADCxConvertedData : %d\r\n" , aADCxConvertedData);

	set_flag(flag_ADC_Data_Ready , ENABLE);
	
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* Clear the A/D interrupt flag */
}

void ADC_ReadAVdd(void)
{
    int32_t  i32ConversionData;
    int32_t  i32BuiltInData;

    ADC_POWER_ON(ADC);
    CLK_SysTickDelay(10000);
	
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, BIT29);
    ADC_SetExtendSampleTime(ADC, 0, 71);
	
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
    ADC_ENABLE_INT(ADC, ADC_ADF_INT);
    NVIC_EnableIRQ(ADC_IRQn);

	set_flag(flag_ADC_Data_Ready ,  DISABLE);
    ADC_START_CONV(ADC);

	while(!is_flag_set(flag_ADC_Data_Ready));

    ADC_DISABLE_INT(ADC, ADC_ADF_INT);
		
    i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 29);
	
    SYS_UnlockReg();
    FMC_Open();
    i32BuiltInData = FMC_ReadBandGap();	

	AVdd = 3072*i32BuiltInData/i32ConversionData;

	printf("%s : %d,%d,%d\r\n",__FUNCTION__,AVdd, i32ConversionData,i32BuiltInData);

    NVIC_DisableIRQ(ADC_IRQn);
	
}

/*
	http://edisonyu71.blogspot.com/2017/08/oversamplingadc.html
	http://ww1.microchip.com/downloads/en/AppNotes/01152A.pdf
	http://ww1.microchip.com/downloads/en/appnotes/doc8003.pdf

	over sampling :
	target : 12bit up to 16 bit , n = 16 - 12

	step 1 : (collect sample , increase 1 bit = 4^n sampling count)
	sampling count : 4^ (16 - 12) = 4^4 = 256 

	step 2: (summary)
	summary the sampling count , 

	step 3: (result)
	and right shift n : summary(256) >> 4
	
*/


uint16_t ADC_ConvertOverSampling(void)
{
	__IO uint16_t adc_value_12bit = 0;
	__IO uint16_t adc_value_16bit = 0;	
	static uint16_t adc_count = 0;
	uint32_t res = 0;

	adc_count = 0;
	oversamplingSum = 0;

//	printf("%s start\r\n",__FUNCTION__);
	
	res = get_tick();
	do
	{
		set_flag(flag_ADC_Data_Ready ,DISABLE);
		ADC_START_CONV(ADC);		
		while(!is_flag_set(flag_ADC_Data_Ready));	

		oversamplingSum += ADC_GET_CONVERSION_DATA(ADC, ADC0_CH0);

//		printf("0x%2X , 0x%6X\r\n" , adc_count , oversamplingSum);
		
	}while (++adc_count < ADC_OVER_SAMPLE_COUNT) ;
	res = get_tick() - res;

	adc_value_16bit = oversamplingSum >> ADC_OVER_SAMPLE_SHIFT;
	adc_value_12bit = oversamplingSum >> 8;	// 2 ^ 8 = 256
	
	printf("OverSample : 0x%4X ,%4dmv (16bit:0x%6X ,%4dmv , 12bit:0x%4X ,%4dmv ) ,timing :%2d\r\n" , 
			oversamplingSum , 
			AVdd ,
			adc_value_16bit  , 
			ADC_CALC_DATA_TO_VOLTAGE_16BIT(adc_value_16bit,AVdd) , 
			adc_value_12bit  , 
			ADC_CALC_DATA_TO_VOLTAGE(adc_value_12bit,AVdd) , 		
			res);
			
	
//	if (adc_value <= ADC_CONVERT_TARGET)
//	{
//		adc_value = ADC_CONVERT_TARGET;
//	}

//	if (adc_value >= ADC_RESOLUTION)
//	{
//		adc_value = ADC_RESOLUTION;
//	}

    /* Stop ADC conversion */
//    ADC_STOP_CONV(ADC);

	return adc_value_16bit;
}

void ADC_InitChannel(uint8_t ch)
{
	set_flag(flag_ADC_Data_Ready , DISABLE);

	ADC_ReadAVdd();

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    /*Wait for ADC internal power ready*/
    CLK_SysTickDelay(10000);

    /* Set input mode as single-end, and Single mode*/
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE,(uint32_t) BIT0);

    /* To sample band-gap precisely, the ADC capacitor must be charged at least 3 us for charging the ADC capacitor ( Cin )*/
    /* Sampling time = extended sampling time + 1 */
    /* 1/24000000 * (Sampling time) = 3 us */
	/*
	    printf("+----------------------------------------------------------------------+\n");
	    printf("|   ADC clock source -> PCLK1  = 48 MHz                                |\n");
	    printf("|   ADC clock divider          = 2                                     |\n");
	    printf("|   ADC clock                  = 48 MHz / 2 = 24 MHz                   |\n");
	    printf("|   ADC extended sampling time = 71                                    |\n");
	    printf("|   ADC conversion time = 17 + ADC extended sampling time = 88         |\n");
	    printf("|   ADC conversion rate = 24 MHz / 88 = 272.7 ksps                     |\n");
	    printf("+----------------------------------------------------------------------+\n");
	*/

    /* Set extend sampling time based on external resistor value.*/
    ADC_SetExtendSampleTime(ADC,(uint32_t) NULL, ADCextendSampling);

    /* Select ADC input channel */
    ADC_SET_INPUT_CHANNEL(ADC, 0x1 << ch);

	ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
	ADC_ENABLE_INT(ADC, ADC_ADF_INT);
	NVIC_EnableIRQ(ADC_IRQn);

    /* Start ADC conversion */
    ADC_START_CONV(ADC);

	while(!is_flag_set(flag_ADC_Data_Ready));

	printf("%s : 0x%4X\r\n" , __FUNCTION__ , ADC_GET_CONVERSION_DATA(ADC, ADC0_CH0));
}



void GPIO_Init (void)
{
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) | (SYS_GPB_MFPH_PB14MFP_GPIO);
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB15MFP_Msk)) | (SYS_GPB_MFPH_PB15MFP_GPIO);
	
    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);	
}


void TMR1_IRQHandler(void)
{
//	static uint32_t LOG = 0;

	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
			PB14 ^= 1;
		}

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res == 'x' || res == 'X')
	{
		NVIC_SystemReset();
	}

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case '1':
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();		
				break;
		}
	}
}

void UART02_IRQHandler(void)
{

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART02_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif	

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);	

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    CLK_EnableModuleClock(ADC_MODULE);	
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PCLK1, CLK_CLKDIV0_ADC(2));	

	/*----------------------------------------------------*/
    SYS->GPB_MFPL = (SYS->GPB_MFPL &~(SYS_GPB_MFPL_PB0MFP_Msk  )) \
                    | (SYS_GPB_MFPL_PB0MFP_ADC0_CH0 ) ;

    /* Set PB.0 ~ PB.3 to input mode */
    GPIO_SetMode(PB, BIT0, GPIO_MODE_INPUT);

    /* Disable the PB0 ~ PB3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0);

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

   /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	UART0_Init();
	GPIO_Init();
	TIMER1_Init();

	ADC_InitChannel(ADC0_CH0);

    /* Got no where to go, just loop forever */
    while(1)
    {
		ADC_ConvertOverSampling();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
