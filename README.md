# M031BSP_ADC_OverSampling
 M031BSP_ADC_OverSampling


update @ 2021/08/12

1. use ADC PB0 (ADC0_CH0) , to test difference over sample flow

2. over sampling :

	target : 
		
		12bit up to 16 bit , n = 16 - 12
	
	step 1 : (collect sample , increase 1 bit = 4^n sampling count)
	
		sampling count : 4^ (16 - 12) = 4^4 = 256 

	step 2: (summary)
	
		summary the sampling count
		
	step 3: (result)

		and right shift n : summary(256) >> 4
		
		
3. below is log message (AVDD high)

![image](https://github.com/released/M031BSP_ADC_OverSampling/blob/main/log_avdd_high.jpg)

![image](https://github.com/released/M031BSP_ADC_OverSampling/blob/main/log_avdd_low.jpg)


4. refer to 

	http://edisonyu71.blogspot.com/2017/08/oversamplingadc.html
	http://ww1.microchip.com/downloads/en/AppNotes/01152A.pdf
	http://ww1.microchip.com/downloads/en/appnotes/doc8003.pdf
	