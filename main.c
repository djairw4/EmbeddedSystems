#include "stm32f4xx.h"
#include "stdio.h"

#define LedOn(PIN) GPIOD->BSRR|=GPIO_BSRR_BS##PIN
#define LedOff(PIN) GPIOD->BSRR|=GPIO_BSRR_BR##PIN
void clock_init(int x);
void systick_init(void);
void DelayMillis(void);
void DelayMs(unsigned long t);
void systick_interrupts(void);
void running_LED(void);
void button_interrupts(void);
void LEDs_enable(void);
void set_PWM(void);
void SPI_init(void);
void SPI_GPIO(void);
void SPI_Enable (void);
void SPI_Disable (void);
void CS_Enable (void);
void CS_Disable (void);
void SPI_Transmit (uint8_t *data, int size);
void SPI_Receive (uint8_t *data, int size);
void L3G4200D_read (uint8_t address);
void L3G4200D_write (uint8_t address, uint8_t value);


static uint8_t clockwise =1;
static uint32_t duty = 0;
static uint8_t flag = 0;

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
				
			//APB1 prescaler = 2
			RCC->CFGR |= RCC_CFGR_PPRE1_2;	
				
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
	SysTick->LOAD = 10000000-1;
	SysTick->VAL = 0;
	SysTick->CTRL=7;
	__enable_irq(); // Enable Interrupts
}

void SysTick_Handler(){
	if(GPIOD->ODR & GPIO_ODR_OD12){
				GPIOD->BSRR |= GPIO_BSRR_BR12;
	}
	else GPIOD->BSRR |= GPIO_BSRR_BS12;
}

void EXTI0_IRQHandler(){
	EXTI->PR |= EXTI_PR_PR0;
	if(clockwise)clockwise=0;
	else clockwise=1;
	//	if(flag){	
//		duty+=10;
//		if(duty>100) duty=0;
//		TIM4->CCR1 = duty;
//		flag=0;
	//}
}

void running_LED(){
	
			if(clockwise){
				if(GPIOD->ODR & GPIO_ODR_OD12){
					GPIOD->BSRR |= GPIO_BSRR_BR12;
					GPIOD->BSRR |= GPIO_BSRR_BS13;
				}
				else if(GPIOD->ODR & GPIO_ODR_OD13){
					GPIOD->BSRR |= GPIO_BSRR_BR13;
					GPIOD->BSRR |= GPIO_BSRR_BS14;
				}
				else if(GPIOD->ODR & GPIO_ODR_OD14){
					GPIOD->BSRR |= GPIO_BSRR_BR14;
					GPIOD->BSRR |= GPIO_BSRR_BS15;
				}
				else if(GPIOD->ODR & GPIO_ODR_OD15){
					GPIOD->BSRR |= GPIO_BSRR_BR15;
					GPIOD->BSRR |= GPIO_BSRR_BS12;
				}
				else{
					GPIOD->BSRR |= GPIO_BSRR_BS15;
				}
				
			}else{
				if(GPIOD->ODR & GPIO_ODR_OD12){
					GPIOD->BSRR |= GPIO_BSRR_BR12;
					GPIOD->BSRR |= GPIO_BSRR_BS15;
				}
				else if(GPIOD->ODR & GPIO_ODR_OD13){
					GPIOD->BSRR |= GPIO_BSRR_BR13;
					GPIOD->BSRR |= GPIO_BSRR_BS12;
				}
				else if(GPIOD->ODR & GPIO_ODR_OD14){
					GPIOD->BSRR |= GPIO_BSRR_BR14;
					GPIOD->BSRR |= GPIO_BSRR_BS13;
				}
				else if(GPIOD->ODR & GPIO_ODR_OD15){
					GPIOD->BSRR |= GPIO_BSRR_BR15;
					GPIOD->BSRR |= GPIO_BSRR_BS14;
				}
				else{
					GPIOD->BSRR |= GPIO_BSRR_BS15;
				}
				
			}			
}

void button_interrupts(){
	__disable_irq(); // Disable Interrupts
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	EXTI->IMR |= EXTI_IMR_MR0;
	EXTI->FTSR |= EXTI_FTSR_TR0;
	NVIC_EnableIRQ(EXTI0_IRQn);
	__enable_irq(); // Enable Interrupts
}

void LEDs_enable(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER &= ~(GPIO_MODER_MODER12);
	//set the pin PD12 as output
	GPIOD->MODER |= GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0 | GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0;
	GPIOD->OTYPER =0;
	GPIOD->OSPEEDR = 0;
}

void set_PWM(void){
	//enable GPIOD
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	//enable TIM4
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	
	//set the pin PD12 -> AF + PP
	GPIOD->MODER &= ~(GPIO_MODER_MODER12);
	GPIOD->MODER |= GPIO_MODER_MODE12_1;
	GPIOD->OTYPER =0;
	GPIOD->OSPEEDR = 0;
	//enable AF2 for PD12 
	GPIOD->AFR[1] |= GPIO_AFRH_AFRH4_1;
	
	TIM4->CCER |= TIM_CCER_CC1E;
	TIM4->CR1 |= TIM_CR1_ARPE;
	TIM4->CCMR1 |= TIM_CCMR1_OC1M_1| TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
	
	//100000000 / PSC / ARR/
	TIM4->PSC = 9999; //max 65536
	TIM4->ARR = 99;  //max 4294967296
	//Duty = CCR1 / ARR 
	TIM4->CCR1 = 0;
	
	TIM4->EGR |= TIM_EGR_UG;
	TIM4->CR1 |= TIM_CR1_CEN;
}



void SPI_init(void){

  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //SPI1 clock enable
	
	SPI1->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA; //CPOL=1 ( CK to 1 when idle), CPHA=1  transmission on rising edge
	
  SPI1->CR1 |= SPI_CR1_MSTR; //Master mode
	
  SPI1->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1; // BR[2:0] = 011: fPCLK/16, PCLK2 = 100MHz, SPI clk = 6,25MHz
	
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST;  // LSBFIRST = 0 -> MSB first D7, D6, D5 ... D0
	
  SPI1->CR1 |= SPI_CR1_SSM| SPI_CR1_SSI;  // SSM=1, SSI=1 -> Software Slave Management
	
  SPI1->CR1 &= ~SPI_CR1_RXONLY;  // RXONLY = 0  -> full-duplex
	
  SPI1->CR1 &= ~SPI_CR1_DFF;  // DFF=0, 8 bit data   
	
  SPI1->CR2 = 0; //disable DMA, interrupts etc.
}

void SPI_GPIO(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOEEN;  // Enable GPIO Clock
	
	GPIOA->MODER |= GPIO_MODER_MODE5_1| GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;  // Alternate functions for PA5, PA6, PA7 
	GPIOE->MODER |= GPIO_MODER_MODE3_0; //Output for PE3
	
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7;//HIGH Speed for PA5, PA6, PA7
	GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEED3; //HIGH Speed for PE3
	
	GPIOA->AFR[0] |= (5<<GPIO_AFRL_AFSEL5_Pos)|(5<<GPIO_AFRL_AFSEL6_Pos)|(5<<GPIO_AFRL_AFSEL7_Pos); // AF5(SPI1) for PA5, PA6, PA7
}

void SPI_Enable (void)
{
	SPI1->CR1 |= SPI_CR1_SPE;   // SPE=1, Peripheral enabled
}

void SPI_Disable (void)
{
	SPI1->CR1 &= ~SPI_CR1_SPE;   // SPE=0, Peripheral Disabled
}

void CS_Enable (void)
{
	GPIOE->BSRR |= GPIO_BSRR_BR3;
}

void CS_Disable (void)
{
	GPIOE->BSRR |= GPIO_BSRR_BS3;
}

void SPI_Transmit (uint8_t *data, int size)
{
	
	/************** STEPS TO FOLLOW *****************
	1. Wait for the TXE bit to set in the Status Register
	2. Write the data to the Data Register
	3. After the data has been transmitted, wait for the BSY bit to reset in Status Register
	4. Clear the Overrun flag by reading DR and SR
	************************************************/		
	
	int i=0;
	while (i<size)
	{
	   while (!((SPI1->SR)&(SPI_SR_TXE))) {}  // wait for TXE bit to set -> This will indicate that the buffer is empty
	   SPI1->DR = data[i];  // load the data into the Data Register
	   i++;
	}	
	
	
/*During discontinuous communications, there is a 2 APB clock period delay between the
write operation to the SPI_DR register and BSY bit setting. As a consequence it is
mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
data.
*/
	while (!((SPI1->SR)&(SPI_SR_TXE))) {}  // wait for TXE bit to set -> This will indicate that the buffer is empty
	while (((SPI1->SR)&(SPI_SR_BSY))) {}  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication	
	
	//  Clear the Overrun flag by reading DR and SR
	uint32_t temp = SPI1->DR;
					temp = SPI1->SR;
	
}

void SPI_Receive (uint8_t *data, int size)
{
	/************** STEPS TO FOLLOW *****************
	1. Wait for the BSY bit to reset in Status Register
	2. Send some Dummy data before reading the DATA
	3. Wait for the RXNE bit to Set in the status Register
	4. Read data from Data Register
	************************************************/		

	while (size)
	{
		while (((SPI1->SR)&(SPI_SR_BSY))) {}  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
		SPI1->DR = 0;  // send dummy data
		while (!((SPI1->SR) &(SPI_SR_RXNE))){}  // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
	  *data++ = (unsigned char)(SPI1->DR);
		size--;
	}	
}

//float xg, yg, zg;
//int16_t x,y,z;
static uint8_t RxData[6];
static uint8_t copyRxData[2];

	
void L3G4200D_write (uint8_t address, uint8_t value)
{
	uint8_t data[2];
	data[0] = address;//|0x40;  // multibyte write
	data[1] = value;
	CS_Enable ();  // pull the cs pin low
	SPI_Transmit (data, 2);  // write data to register
	CS_Disable ();  // pull the cs pin high
}
	

void L3G4200D_read (uint8_t address)
{
	address |= 0x80;  // read operation
	address |= 0x40;  // multibyte read
	CS_Enable ();  // pull the pin low
	SPI_Transmit (&address, 1);  // send address
	SPI_Receive (RxData, 6);  // receive 6 bytes data
	CS_Disable ();  // pull the pin high
}

int main(void){
//	clock_init(0);
//	SystemCoreClockUpdate();
//  printf("Clock frequency: %u Hz\n",SystemCoreClock);

	
	clock_init(2);
	SystemCoreClockUpdate();
	//SysTick_Config(SystemCoreClock/100000);

	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	//GPIOA->MODER &= ~(GPIO_MODER_MODER0);
	

	//LEDs_enable();
	systick_init();
	//systick_interrupts();
	//button_interrupts();
	//set_PWM();
	
	SPI_GPIO();
	SPI_init();
	SPI_Enable();
	//L3G4200D_read(0x20);
	//copyRxData[0]=RxData[0];
	//copyRxData[1]=RxData[1];
	//L3G4200D_write (0x20, 0x0F);
	L3G4200D_write (0x20, 0x47);
	
	while(1){
		//if(GPIOA->IDR & GPIO_IDR_IDR_0){clock_init(1);SystemCoreClockUpdate();} 
		//GPIOD->ODR |= GPIO_ODR_ODR_12;  // Set the pin PD12
		//GPIOD->ODR &= ~(GPIO_ODR_ODR_12);  // Reset pin PD12

		//running_LED();
		//L3G4200D_read(0x0F);
		
		
		DelayMs(1000);
		L3G4200D_read(0x26);
		

	}
}
