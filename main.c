#include "stm32l1xx.h"
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
	
	uint32_t ADC_value, ADC_result, step= ;
	
	RCC->AHBENR|=RCC_AHBENR_GPIOBEN; //LED blue and green
	RCC->AHBENR|=RCC_AHBENR_GPIOAEN; //ADC, MCO, USART 
	

	
	//USART2 clocks
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;	
	
	RCC->CR|=RCC_CR_HSION; //HSI on
	RCC->CFGR|=RCC_CFGR_MCO_DIV2|RCC_CFGR_MCO_SYSCLK|RCC_CFGR_SW_0|RCC_CFGR_PPRE1_DIV1; //MCO config
	
	//ADC clocks
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	//ADC init
	GPIOA->MODER |=GPIO_MODER_MODER1; // analog
	
	//LED init
	GPIOB->MODER|=GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0; // PB7=01, PB6=01 output
  GPIOB->OTYPER|=0; //push-pull 
	GPIOB->OSPEEDR|=0; // Low Speed 
	GPIOB->PUPDR|=GPIO_PUPDR_PUPDR6_1|GPIO_PUPDR_PUPDR7_1;// pull-down
	
	//MCO init
	GPIOA->MODER|=GPIO_MODER_MODER8_1; //PA8=10 AF
  GPIOA->OTYPER|=0; //push-pull 
	GPIOA->PUPDR|=~GPIO_PUPDR_PUPDR8;// no pull
	GPIOA->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR8; // High Speed 
	GPIOA->AFR[1]|=(GPIO_AFRH_AFRH8>>4);
	
	//GPIO usart init
	GPIOA->MODER|=GPIO_MODER_MODER2_1|GPIO_MODER_MODER3_1; //PA2=10, PA3=10 AF
  GPIOA->OTYPER|=0; //push-pull 
	GPIOA->PUPDR|=~GPIO_PUPDR_PUPDR2|~GPIO_PUPDR_PUPDR3;// no pull
	GPIOA->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR2|GPIO_OSPEEDER_OSPEEDR3; // High Speed 
	GPIOA->AFR[0]|=((GPIO_AFRL_AFRL2 & (0x00000007<<8))|(GPIO_AFRL_AFRL3 & (0x00000007<<8))); //PA2, PA3 AF7
	
	//ADC config
	ADC1->SMPR3 &= ADC_SMPR3_SMP1; //write when ADON=0
	ADC1->CR1 &= (~ADC_CR1_RES)|ADC_CR1_SCAN; //res 12 bit, scan mode enabled
	ADC1->CR2 |= (ADC_CR2_CONT|ADC_CR2_ADON) & (~ADC_CR2_ALIGN); //continuous conv mode, right alignment, ADC on
	ADC1->JSQR &= ~ADC_JSQR_JL&(~ADC_JSQR_JSQ4_0); //1 conversion, 1 channel
	ADC1->CR2 |= ADC_CR2_JSWSTART; //start ADC
	
	
	  //USART config
		USART2->CR1 |= USART_CR1_UE; //Включени?USART
    USART2->CR1 |= USART_CR1_M;  // 8 би?данных
    USART2->CR2 &=~ USART_CR2_STOP; //Один стоповый би?
    USART2->BRR =0x8b  ;//0x8B; APBCLK/BAUDRATE // Скорость 115200 бо?
		USART2->CR1 &=~ USART_CR1_PCE; //Запретит?би?четности
    USART2->CR1 |= USART_CR1_TE; //Включить предатчи?USART
    USART2->CR1 |= USART_CR1_RE; //Включить приемник USART
		
		
  	// \n - Переместит?позици?печати на одну строку вниз
		// \r - Переместит?позици?печати ?лево?крайне?подожени?
		SendUSART((uint8_t *)" _____________________________  \n\r");
		SendUSART((uint8_t *)"|                             | \n\r");
    SendUSART((uint8_t *)"|   Developed by Yalaletdinov | \n\r");
		SendUSART((uint8_t *)"|_____________________________| \n\r");
		SendUSART((uint8_t *)"|                             | \n\r");
    SendUSART((uint8_t *)"|STM32l152RCT6 ready for work | \n\r");
		SendUSART((uint8_t *)"|_____________________________| \n\r");
    
while(1)
{   
	  if  (ADC1->SR & ADC_SR_JEOC)
		{			
	  ADC_value = ADC1->JDR4;
	  ADC_result = ADC_value * step;
	  }
		
	  for (i=0;i<3000000;++i) {};
    temp=USART2->DR;
    if 	(temp=='1')
    {
    GPIOB->ODR|=GPIO_ODR_ODR_6;  
		}		
	
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
			}

}		
	
}


void SendUSART (uint8_t *text)
{
	while(*text)
	{
		while(!(USART2->SR & USART_SR_TC)); //Би?готовности передатчик?
		USART2->DR = *text;	//Записать данные ?регист?передачи
		text++;		
	}
}

uint8_t TakeUSART (void)
{
	uint8_t data;
	
	if (USART2->SR & USART_SR_RXNE) //Би?устанавливается ?1, когд?регист?RDR поло?(пришли данные)
		{
			data = USART2->DR; //Считат?данные из регистра приема
		}
		
		return data;
}

void LED_GREEN_ON (void)
{
	GPIOB->ODR|=GPIO_ODR_ODR_7;
}

void LED_GREEN_OFF (void)
{
	GPIOB->ODR|=GPIO_ODR_ODR_7&(~GPIO_ODR_ODR_7);
}

void LED_BLUE_ON (void)
{
	GPIOB->ODR|=GPIO_ODR_ODR_6;
}

void LED_BLUE_OFF (void)
{
	GPIOB->ODR|=GPIO_ODR_ODR_6&(~GPIO_ODR_ODR_6);
}


