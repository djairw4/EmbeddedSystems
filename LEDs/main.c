#include "stm32f4xx.h"
#include "stdio.h"

#define LedOn(PIN) GPIOD->BSRR|=GPIO_BSRR_BS##PIN
#define LedOff(PIN) GPIOD->BSRR|=GPIO_BSRR_BR##PIN
void clock_init(int x);
void GPIO_Config (void);
void delay (uint32_t time);
void delayMs (int ms);
void systick_init(void);
void DelayMillis(void);
void DelayMs(unsigned long t);
extern void EXTI0_IRQHandler ();


uint8_t clockwise =1;
static int msTicks = 0;

//clock source
void clock_init(int x){
	switch(x){
		//HSI - 16MHz
		case 0: 
			//turn on HSI
			RCC->CR |= RCC_CR_HSION;
			while(!(RCC->CR & RCC_CR_HSIRDY)){}
			//set HSI as clock source
			RCC->CFGR &= ~(RCC_CFGR_SW);
			while(RCC->CFGR & RCC_CFGR_SWS){}
				
			FLASH->ACR &= ~(FLASH_ACR_LATENCY);	
			break;
		//HSE	- 8MHz	
		case 1:
			//turn on HSE
			RCC->CR |= RCC_CR_HSEON;
			//waiting for external crystal be stable
			while(!(RCC->CR & RCC_CR_HSERDY)){}	
			//set HSE as clock source	
			RCC->CFGR &= ~(RCC_CFGR_SW);
			RCC->CFGR |= RCC_CFGR_SW_0;	
			while(!(RCC->CFGR & RCC_CFGR_SWS_0)){}
				
			FLASH->ACR &= ~(FLASH_ACR_LATENCY);	
			break;
		//PLL - 100MHz		
		case 2:
			//turn on HSE
			RCC->CR |= RCC_CR_HSEON;
			while(!(RCC->CR & RCC_CR_HSERDY)){}
				
				
			RCC->AHB1ENR |= RCC_APB1ENR_PWREN;
			PWR->CR |= PWR_CR_VOS	;
			
			//Instruction prefetch enable
			FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_DCEN | FLASH_ACR_ICEN;
			
			// set 3WS (wait states) -> (90MHz;120MHz>
			FLASH->ACR &= ~(FLASH_ACR_LATENCY);
			FLASH->ACR |= FLASH_ACR_LATENCY_3WS;
				
			
			RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
			RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
			RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
				
		
			//external clock as PLL source - 8MHz
			RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
				
			//reset values of  PLLM, PLLN, PLLP
			RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP);
			
			//PLLM <2;63>, VCOinput <1MHz;2MHz>
			//PLLM = 4   =>  VCOinput = 8MHz / 4 = 2MHz
			RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_2;
			
			//PLLN <50;432>, VCOoutput <100MHz;432MHz>
			//PLLN = 100   =>  VCOinput = 2MHz * 100 = 200MHz
			RCC->PLLCFGR |= (RCC_PLLCFGR_PLLN_2 | RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_6);
				
			//PLLP = 2  => SYSCLK = 200MHz / 2 = 100MHz	 
				
			//turn on PLL
			RCC->CR |= RCC_CR_PLLON;
			while(!(RCC->CR & RCC_CR_PLLRDY)){}
	
			//set PLL as clock source	
			RCC->CFGR &= ~(RCC_CFGR_SW);
			RCC->CFGR |= RCC_CFGR_SW_1;	
			while(!(RCC->CFGR & RCC_CFGR_SWS_1)){}		

			break;
		default: break;		
	}
	
	
}

void systick_init(void){
	SysTick->CTRL=0;
	SysTick->LOAD = 0x00FFFFFF;
	SysTick->VAL = 0;
	SysTick->CTRL=5;
}

void DelayMillis(void){
		SysTick->LOAD=100000-1;
		SysTick->VAL = 0;
		while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
}

void DelayMs(unsigned long t){
	for(;t>0;t--){
		DelayMillis();
	}
}

void systick_interrupts(void){
	__disable_irq(); // Disable Interrupts
	SysTick->CTRL=0;
	SysTick->LOAD = 1000000-1;
	SysTick->VAL = 0;
	SysTick->CTRL=7;
	__enable_irq(); // Enable Interrupts
}

void Systick_Handler(){
	if(GPIOD->ODR & GPIO_ODR_OD12){
				GPIOD->BSRR |= GPIO_BSRR_BR12;
	}
	else GPIOD->BSRR |= GPIO_BSRR_BS12;
}

void EXTI0_IRQHandler(){
	EXTI->PR |= EXTI_PR_PR0;
	if(clockwise)clockwise=0;
	else clockwise=1;
}


int main(void){
//	clock_init(0);
//	SystemCoreClockUpdate();
//  printf("Clock frequency: %u Hz\n",SystemCoreClock);

	
	clock_init(2);
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/100000);

	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	//GPIOA->MODER &= ~(GPIO_MODER_MODER0);
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER &= ~(GPIO_MODER_MODER12);
	//set the pin PD12 as output
	GPIOD->MODER |= GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0 | GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0;
	GPIOD->OTYPER =0;
	GPIOD->OSPEEDR = 0;
	
	//systick_init();
	systick_interrupts();
	 __disable_irq(); // Disable Interrupts
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	EXTI->IMR |= EXTI_IMR_MR0;
	EXTI->FTSR |= EXTI_FTSR_TR0;
	NVIC_EnableIRQ(EXTI0_IRQn);
	__enable_irq(); // Enable Interrupts
	
	
	
	//uint8_t led_pin =12;
	while(1){
		//if(GPIOA->IDR & GPIO_IDR_IDR_0){clock_init(1);SystemCoreClockUpdate();} 
		//GPIOD->ODR |= GPIO_ODR_ODR_12;  // Set the pin PD12
		//GPIOD->ODR &= ~(GPIO_ODR_ODR_12);  // Reset pin PD12
//		GPIOD->BSRR |= GPIO_BSRR_BS12;
//		DelayMs(1000);
//		GPIOD->BSRR |= GPIO_BSRR_BR12;
//		DelayMs(1000);
//			if(clockwise){
//				GPIOD->BSRR |= GPIO_BSRR_BS12;
//				GPIOD->BSRR |= GPIO_BSRR_BR15;
//				DelayMs(1000);		
//				GPIOD->BSRR |= GPIO_BSRR_BS13;
//				GPIOD->BSRR |= GPIO_BSRR_BR12;
//				DelayMs(1000);
//				GPIOD->BSRR |= GPIO_BSRR_BS14;
//				GPIOD->BSRR |= GPIO_BSRR_BR13;
//				DelayMs(1000);
//				GPIOD->BSRR |= GPIO_BSRR_BS15;
//				GPIOD->BSRR |= GPIO_BSRR_BR14;
//				DelayMs(1000);
//			}else{
//				GPIOD->BSRR |= GPIO_BSRR_BS15;
//				GPIOD->BSRR |= GPIO_BSRR_BR12;
//				DelayMs(1000);		
//				GPIOD->BSRR |= GPIO_BSRR_BS14;
//				GPIOD->BSRR |= GPIO_BSRR_BR15;
//				DelayMs(1000);
//				GPIOD->BSRR |= GPIO_BSRR_BS13;
//				GPIOD->BSRR |= GPIO_BSRR_BR14;
//				DelayMs(1000);
//				GPIOD->BSRR |= GPIO_BSRR_BS12;
//				GPIOD->BSRR |= GPIO_BSRR_BR13;
//				DelayMs(1000);
//			}

//			if(clockwise){
//				if(GPIOD->ODR & GPIO_ODR_OD12){
//					GPIOD->BSRR |= GPIO_BSRR_BR12;
//					GPIOD->BSRR |= GPIO_BSRR_BS13;
//				}
//				else if(GPIOD->ODR & GPIO_ODR_OD13){
//					GPIOD->BSRR |= GPIO_BSRR_BR13;
//					GPIOD->BSRR |= GPIO_BSRR_BS14;
//				}
//				else if(GPIOD->ODR & GPIO_ODR_OD14){
//					GPIOD->BSRR |= GPIO_BSRR_BR14;
//					GPIOD->BSRR |= GPIO_BSRR_BS15;
//				}
//				else if(GPIOD->ODR & GPIO_ODR_OD15){
//					GPIOD->BSRR |= GPIO_BSRR_BR15;
//					GPIOD->BSRR |= GPIO_BSRR_BS12;
//				}
//				else{
//					GPIOD->BSRR |= GPIO_BSRR_BS15;
//				}
//				
//			}else{
//				if(GPIOD->ODR & GPIO_ODR_OD12){
//					GPIOD->BSRR |= GPIO_BSRR_BR12;
//					GPIOD->BSRR |= GPIO_BSRR_BS15;
//				}
//				else if(GPIOD->ODR & GPIO_ODR_OD13){
//					GPIOD->BSRR |= GPIO_BSRR_BR13;
//					GPIOD->BSRR |= GPIO_BSRR_BS12;
//				}
//				else if(GPIOD->ODR & GPIO_ODR_OD14){
//					GPIOD->BSRR |= GPIO_BSRR_BR14;
//					GPIOD->BSRR |= GPIO_BSRR_BS13;
//				}
//				else if(GPIOD->ODR & GPIO_ODR_OD15){
//					GPIOD->BSRR |= GPIO_BSRR_BR15;
//					GPIOD->BSRR |= GPIO_BSRR_BS14;
//				}
//				else{
//					GPIOD->BSRR |= GPIO_BSRR_BS15;
//				}
//				
//			}				
		

		//DelayMs(300);

	}
}