
#include "stm32l1xx.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define APBCLK   16000000UL
#define BAUDRATE 115200UL
#define TS_cal1 ((uint32_t*) 0x1FF800FA)
#define TS_cal11 ((uint32_t*) 0x1FF800FB)
#define TS_cal2 ((uint32_t*) 0x1FF800FE)
#define TS_cal22 ((uint32_t*) 0x1FF800FF)

#define Vref_int_cal1 ((uint16_t*) 0x1FF800F8)
#define Vref_int_cal11 ((uint16_t*) 0x1FF800F9)

#define led_green_on  '1'
#define led_green_off '2'
#define led_blue_on   '3'
#define led_blue_off  '4'


void SendUSART (uint8_t *text);
uint8_t TakeUSART  (void); 
void LED_GREEN_ON  (void);
void LED_GREEN_OFF (void);
void LED_BLUE_ON   (void);
void LED_BLUE_OFF  (void);

int main()
{
	uint32_t i, temp;
	uint8_t command;
	
	
	uint32_t ADC_value, ADC_result, a, DAC_result;
  uint8_t b=0;
	char txt_buf[200];
	char DAC_buf[00];
	
	uint32_t n[10] = 
{
  '0', //0
  '1', //1
  '2', //2
  '3', //3   
  '4', //4
  '5', //5 
  '6', //6
  '7', //7   
  '8', //8
  '9',  //9  
  };
	

	RCC->AHBENR|=RCC_AHBENR_GPIOBEN; //LED blue and green
	RCC->AHBENR|=RCC_AHBENR_GPIOAEN; //ADC, MCO, USART 
	

	
	//USART2 clocks
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;	
	
	RCC->CR|=RCC_CR_HSION; //HSI on
	RCC->CFGR|=RCC_CFGR_MCO_DIV2|RCC_CFGR_MCO_SYSCLK|RCC_CFGR_SW_0|RCC_CFGR_PPRE1_DIV1; //MCO config
	
	//ADC clocks
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  //DAC clocks
  RCC->APB1ENR |= RCC_APB1ENR_DACEN; 
			
	//GPIO LED init
	GPIOB->MODER|=GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0; // PB7=01, PB6=01 output
  GPIOB->OTYPER|=0; //push-pull 
	GPIOB->OSPEEDR|=0; // Low Speed 
	GPIOB->PUPDR|=GPIO_PUPDR_PUPDR6_1|GPIO_PUPDR_PUPDR7_1;// pull-down
	
	//GPIO MCO init
	GPIOA->MODER|=GPIO_MODER_MODER8_1; //PA8=10 AF
  GPIOA->OTYPER|=0; //push-pull 
	GPIOA->PUPDR &=~ GPIO_PUPDR_PUPDR8;// no pull
	GPIOA->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR8; // High Speed 
	GPIOA->AFR[1]|=(GPIO_AFRH_AFRH8>>4);
	
	//GPIO USART init
	GPIOA->MODER|=GPIO_MODER_MODER2_1|GPIO_MODER_MODER3_1; //PA2=10, PA3=10 AF
  GPIOA->OTYPER|=0; //push-pull 
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR2 & ~GPIO_PUPDR_PUPDR3;// no pull
	GPIOA->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR2|GPIO_OSPEEDER_OSPEEDR3; // High Speed 
	GPIOA->AFR[0]|=((GPIO_AFRL_AFRL2 & (0x00000007<<8))|(GPIO_AFRL_AFRL3 & (0x00000007<<12))); //PA2, PA3 AF7
	
	
	//GPIO ADC init
	GPIOA->MODER |=GPIO_MODER_MODER1; // analog function ch1, PA1
	
	//GPIO DAC init
	//GPIOA->MODER|=GPIO_MODER_MODER4|GPIO_MODER_MODER5; //PA4, PA5 Analog Function
  //GPIOA->OTYPER|=0; //push-pull 
	//GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4 & ~GPIO_PUPDR_PUPDR5;// no pull
	//GPIOA->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR4|GPIO_OSPEEDER_OSPEEDR5; // High Speed 
	
		
	//USART config
	USART2->CR1 |= USART_CR1_UE; //USART on
  USART2->CR1 |= USART_CR1_M;  // 8 bit
  USART2->CR2 &=~ USART_CR2_STOP; //1 stop bit
  USART2->BRR =0x8b ; //APBCLK/BAUDRATE // 115200 baud
	USART2->CR1 &=~ USART_CR1_PCE; // parity bit disabled
  USART2->CR1 |= USART_CR1_TE; // USART transmitter  on 
  USART2->CR1 |= USART_CR1_RE; //USART receiver on
	
	//DAC config
  DAC->CR|=DAC_CR_EN1;
	//DAC->CR|=DAC_CR_TSEL1; // 111-software trigger
	
	
	
	//ADC config, injected channel
	ADC1->SMPR3 &= ADC_SMPR3_SMP1; //sample time 384 cycles, write when ADON=0 
	ADC1->CR1 &= ~ADC_CR1_RES; //res 12 bit
	ADC1->CR1 &= ~ADC_CR1_SCAN; //scan mode disabled
	
	ADC1->CR2 &= ~ADC_CR2_CONT; //single conv mode
	ADC1->CR2 |= ADC_CR2_ADON; //ADC on
	ADC1->CR2 &= ~ADC_CR2_ALIGN; //right alignment
	ADC1->CR2 &= ~ADC_CR2_CFG; //bank A 
		
	ADC1->JSQR &= ~ADC_JSQR_JL; //1 conversion
	ADC1->JSQR |= ADC_JSQR_JSQ4_0; //1st conversion in injected channel
	
	
	//regular channel
	/*
	ADC1->SQR1 &= ~ADC_SQR1_L; //1 conversion
	ADC1->SQR5 |= ADC_SQR5_SQ1_0; //1st conversion in regular channel
	*/
		
		
  	// \n - переместить курсор на строку вниз 
		// \r - переместить курсор в крайнее левое положение
		
		SendUSART((uint8_t *)" _____________________________  \n\r");
		SendUSART((uint8_t *)"|                             | \n\r");
    SendUSART((uint8_t *)"|   Developed by Yalaletdinov | \n\r");
		SendUSART((uint8_t *)"|_____________________________| \n\r");
		SendUSART((uint8_t *)"|                             | \n\r");
    SendUSART((uint8_t *)"|STM32l152RCT6 ready for work | \n\r");
		SendUSART((uint8_t *)"|_____________________________| \n\r");
    SendUSART((uint8_t *)"Чтобы обновить напряжение на PA1 нажмите R \n\r");
while(1)
{   
	
	//
	
	
	ADC1->CR2 |= ADC_CR2_JSWSTART; //start ADC, inj ch
	//ADC1->CR2 |= ADC_CR2_SWSTART; //start ADC, reg ch
	    
	  
	  if  (ADC1->SR & ADC_SR_JEOC) //wait of JEOC
		{			
	  ADC_value = ADC1->JDR1;
	  ADC_result = (ADC_value * 3000)/4095;
	  }
		sprintf (txt_buf, "\n\rНапряжение на АЦП U=%d мВ",ADC_result);
		sprintf (txt_buf, "\n\rНапряжение на АЦП U=%d мВ",ADC_result);
		
	  //sprintf (txt_buf, "\n\r TS_cal1=%d TS_cal11=%d TS_cal2=%d TS_cal22=%d Vref_int_cal1=%d Vref_int_cal11=%d ",*TS_cal1, *TS_cal11, *TS_cal2, *TS_cal22, *Vref_int_cal1, *Vref_int_cal11);
		//sprintf (txt_buf, "\n\r TS_cal1=%d TS_cal11=%d TS_cal2=%d  Vref_int_cal1=%d Vref_int_cal11=%d ",* TS_cal1, * TS_cal11, * TS_cal2,  *Vref_int_cal1, *Vref_int_cal11);
		sprintf (txt_buf, "\n\rVref_int_cal1=%d Vref_int_cal11=%d ", *Vref_int_cal1, *Vref_int_cal11);	
		SendUSART ((uint8_t*) txt_buf); 
		/*
		if  (ADC1->SR & ADC_SR_EOC) //wait of EOC
		{			
	  ADC_value = ADC1->DR;
	  ADC_result = (ADC_value * 3000)/4095;
	  }
		*/
		for (i=0;i<5000000;++i) {};
		//DAC->SWTRIGR|=DAC_SWTRIGR_SWTRIG1; // software trigger enabled		
		DAC->DHR12R1=3095;
		DAC_result = DAC->DHR12R1;
		sprintf (DAC_buf, "\n\rКод ЦАП %d ",DAC->DOR1);
		//sscanf(DAC_buf,"%d",&DAC_result);
		//SendUSART ((uint8_t*) txt_buf);
		//SendUSART ((uint8_t*) DAC_buf);
	 
	
	for (i=0;i<500000;++i) {};
	
	command = TakeUSART();
			switch(command)
			{
				case led_green_on:
				  LED_GREEN_ON();
				  SendUSART((uint8_t *)"Led green ON \n\r");
				break;
				case led_green_off:
					LED_GREEN_OFF();
				  SendUSART((uint8_t *)"Led green OFF \n\r");
				break;
				case led_blue_on:
					LED_BLUE_ON();
				  SendUSART((uint8_t *)"Led blue ON \n\r");
				break;
				case led_blue_off:
					LED_BLUE_OFF();
				  SendUSART((uint8_t *)"Led blue OFF \n\r");	
				break;
				case 'r':
				SendUSART ((uint8_t*) txt_buf); 
				break;
				case 'R':
				SendUSART ((uint8_t*) txt_buf); 
				break;
				case 'z':
				//sprintf (txt_buf, "\n\r TS_cal1=%d TS_cal11=%d TS_cal2=%d TS_cal22=%d Vref_int_cal1=%d Vref_int_cal11=%d ",* TS_cal1, * TS_cal11, * TS_cal2, * TS_cal22, *Vref_int_cal1, *Vref_int_cal11);
				SendUSART ((uint8_t*) txt_buf); 
				break;
			}

}		
	
}


void SendUSART (uint8_t *text)
{
	while(*text)
	{
		while(!(USART2->SR & USART_SR_TC)); //Transmission is complete
		USART2->DR = *text;	//write data
		text++;		
	}
}

uint8_t TakeUSART (void)
{
	uint8_t data;
	
	if (USART2->SR & USART_SR_RXNE) //Received data is ready to be read
		{
			data = USART2->DR; //read data
		}
		
		return data;
}

void LED_GREEN_ON (void)
{
	GPIOB->ODR|=GPIO_ODR_ODR_7;
}

void LED_GREEN_OFF (void)
{
	GPIOB->BSRRH|=GPIO_BSRR_BS_7; //reset PB7 to low level
}

void LED_BLUE_ON (void)
{
	GPIOB->ODR|=GPIO_ODR_ODR_6;
}

void LED_BLUE_OFF (void)
{
	GPIOB->BSRRH|=GPIO_BSRR_BS_6; ////reset PB6 to low level
}


