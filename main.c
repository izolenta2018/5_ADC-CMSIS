#include "stm32l1xx.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define APBCLK   16000000UL
#define BAUDRATE 115200UL

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
	
	
	uint32_t ADC_value, ADC_result, a;
  uint8_t b=0;
	char txt_buf[150];
	
	
	uint32_t n[11] = 
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
  'x'
};
	

	RCC->AHBENR|=RCC_AHBENR_GPIOBEN; //LED blue and green
	RCC->AHBENR|=RCC_AHBENR_GPIOAEN; //ADC, MCO, USART 
	

	
	//USART2 clocks
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;	
	
	RCC->CR|=RCC_CR_HSION; //HSI on
	RCC->CFGR|=RCC_CFGR_MCO_DIV2|RCC_CFGR_MCO_SYSCLK|RCC_CFGR_SW_0|RCC_CFGR_PPRE1_DIV1; //MCO config
	
	//ADC clocks
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
			
	//GPIO LED init
	GPIOB->MODER|=GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0; // PB7=01, PB6=01 output
  GPIOB->OTYPER|=0; //push-pull 
	GPIOB->OSPEEDR|=0; // Low Speed 
	GPIOB->PUPDR|=GPIO_PUPDR_PUPDR6_1|GPIO_PUPDR_PUPDR7_1;// pull-down
	
	//GPIO MCO init
	GPIOA->MODER|=GPIO_MODER_MODER8_1; //PA8=10 AF
  GPIOA->OTYPER|=0; //push-pull 
	GPIOA->PUPDR|=~GPIO_PUPDR_PUPDR8;// no pull
	GPIOA->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR8; // High Speed 
	GPIOA->AFR[1]|=(GPIO_AFRH_AFRH8>>4);
	
	//GPIO USART init
	GPIOA->MODER|=GPIO_MODER_MODER2_1|GPIO_MODER_MODER3_1; //PA2=10, PA3=10 AF
  GPIOA->OTYPER|=0; //push-pull 
	GPIOA->PUPDR|=~GPIO_PUPDR_PUPDR2|~GPIO_PUPDR_PUPDR3;// no pull
	GPIOA->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR2|GPIO_OSPEEDER_OSPEEDR3; // High Speed 
	GPIOA->AFR[0]|=((GPIO_AFRL_AFRL2 & (0x00000007<<8))|(GPIO_AFRL_AFRL3 & (0x00000007<<12))); //PA2, PA3 AF7
		
	//USART config
	USART2->CR1 |= USART_CR1_UE; //USART on
  USART2->CR1 |= USART_CR1_M;  // 8 bit
  USART2->CR2 &=~ USART_CR2_STOP; //1 stop bit
  USART2->BRR =0x8b ; //APBCLK/BAUDRATE // 115200 baud
	USART2->CR1 &=~ USART_CR1_PCE; // parity bit disabled
  USART2->CR1 |= USART_CR1_TE; // USART transmitter  on 
  USART2->CR1 |= USART_CR1_RE; //USART receiver on
	
	//GPIO ADC init
	GPIOA->MODER |=GPIO_MODER_MODER1; // analog
	
	//ADC config
	ADC1->SMPR3 &= ADC_SMPR3_SMP1; //write when ADON=0
	ADC1->CR1 &= ~ADC_CR1_RES; //res 12 bit, scan mode enabled
	//ADC1->CR1 |= ADC_CR1_SCAN; //res 12 bit, scan mode enabled
	ADC1->CR1 &= ~ADC_CR1_SCAN; //res 12 bit, scan mode disabled
	
	ADC1->CR2 &= ~ADC_CR2_CONT; //single conv mode, right alignment, ADC on
	//ADC1->CR2 |=ADC_CR2_CONT; //continuous conv mode, right alignment, ADC on
	
	ADC1->CR2 |= ADC_CR2_ADON; //single conv mode, right alignment, ADC on
	ADC1->CR2 &= ~ADC_CR2_ALIGN; //single conv mode, right alignment, ADC on
	
	
	ADC1->CR2 &= ~ADC_CR2_CFG; //bank A 
	//ADC1->CR2 |= ADC_CR2_CFG; //bank B 
	
	
	ADC1->JSQR &= ~ADC_JSQR_JL; //1 conversion, 1 channel
	ADC1->JSQR |= ADC_JSQR_JSQ4_0; //1 conversion, 1 channel
	
	
	/*
	ADC1->SQR1 &= ~ADC_SQR1_L; //1 conversion, 1 channel
	ADC1->SQR5 |= ADC_SQR5_SQ1_0; //1 conversion, 1 channel
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
	
	ADC1->CR2 |= ADC_CR2_JSWSTART; //start ADC
	//ADC1->CR2 |= ADC_CR2_SWSTART; //start ADC
	 //for (i=0;i<500000;++i) {};
    
	  
	  if  (ADC1->SR & ADC_SR_JEOC)
		{			
	  ADC_value = ADC1->JDR1;
	  ADC_result = (ADC_value * 3000)/4095;
	  }
		
		sprintf (txt_buf, "\n\rНапряжение на АЦП U=%d мВ",ADC_result);
		//SendUSART ((uint8_t*) txt_buf);
	
		
		if  (ADC1->SR & ADC_SR_EOC)
		{			
	  ADC_value = ADC1->DR;
	  ADC_result = (ADC_value * 3000)/4095;
	  }
		
		/*
		// Decomposition of number a=7964
		a=ADC_result/1000;        //7
    ADC_result%=1000;        //964
    
		while(!(USART2->SR & USART_SR_TC)); //Transmission is complete
		USART2->DR =n[a];	//write data
		
    a=ADC_result/100;          //9
    ADC_result%=100;          //64
		
		while(!(USART2->SR & USART_SR_TC)); //Transmission is complete
		USART2->DR =n[a];	//write data

    a=ADC_result/10;           //6
		
		while(!(USART2->SR & USART_SR_TC)); //Transmission is complete
		USART2->DR =n[a];	//write data

    a=ADC_result%10;         //4
		*/
	
	//for (i=0;i<50000;++i) {};
	
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


