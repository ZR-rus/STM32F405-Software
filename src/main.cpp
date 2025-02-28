#include<stm32f405xx.h>

void setup_core_STM32F405 ()
{
  RCC->CR |= (RCC_CR_HSEON); 							// тактировать от внешнего кварца
	while( !(RCC->CR & RCC_CR_HSERDY) ) {}  // ожидание подключения кварца
	//FLASH
	FLASH->ACR |= FLASH_ACR_LATENCY | FLASH_ACR_PRFTEN| FLASH_ACR_ICEN| FLASH_ACR_DCEN;
	//PLL - HSE
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE; //PLL работает от внешнего кварца
	RCC->CR &= ~(RCC_CR_PLLON); 						//отключаем PLL перед настройкой
  ////////////////////////////// настройка PLL ////////////////////////////////
  //f(VCO clock) = f(PLL clock input) × (PLLN / PLLM)
  //f(PLL general clock output) = f(VCO clock) / PLLP
  //f(USB OTG FS, SDIO, RNG clock output) = f(VCO clock) / PLLQ
  /////////////////////////////////////////////////////////////////////////////
	//PLL M(делитель)
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM); 		//очистить все биты PLLM 
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_2; 		// PLLM = 4 (00100)
	//PLL N(множитель_1)
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN); 		//очистить все биты PLLN
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_3; 		//PLLN = 168 (1010 1000)
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_5; 		//
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_7; 		//
  //получили f(VCO clock) = 336 для кварца 8Мгц
  //PLL P(делитель_1)
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP); 		//очистить все биты, PLLP = 2 (00)
  //получили f(PLL general clock output) = 168 для кварца 8Мгц
	//PLL Q (делитель_2) 
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLQ);    //очистить все биты PLLQ
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_0;     //PLLQ =7 (0111)
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_1;
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_2;
  //получили f(USB OTG FS, SDIO, RNG clock output) = 48 для кварца 8Мгц
  ///////////////////////////// настройка шин /////////////////////////////////
	//AHB Prescaler
	RCC->CFGR &= ~(RCC_CFGR_HPRE); 					//очистить все биты AHB 
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1; 		  	//AHB = SYSCLK/1 (168Мгц)
	//APB1 Prescaler 
	RCC->CFGR &= ~(RCC_CFGR_PPRE1);         //очистить все биты APB1 
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;       // APB1 = 4 (42Мгц) 
	//APB2 Prescaler 
	RCC->CFGR &= ~(RCC_CFGR_PPRE2);         //очистить все биты APB2 
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;       // APB2 = 2 (84Мгц)
	/////////////////////////////////////////////////////////////////////////////
	RCC->CR |= RCC_CR_PLLON; 								//включаем PLL
	while((RCC->CR & RCC_CR_PLLRDY) == 0) {}//ожидаем включения PLL
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL; 					//выбрать PLL для тактирования контроллера
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {} 	//ожидание начала тактирования PLL 
  //__enable_irq ();
  //////////////////////////////////// настройка портов /////////////////////////////////////////////////
  RCC->AHB1ENR |=RCC_AHB1ENR_GPIOBEN;	// тактирование GPIOB
  GPIOB->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 |GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 ; // PB0,PB1,PB6,PB7 на выход
  GPIOB->OSPEEDR |=GPIO_OSPEEDR_OSPEED7_0; // скорость порта 25 Mhz
}

void timer_1 ()
{
  //NVIC->ISER[0] |= (1<<27);                                              // разрешить прерывания в таймере T1 при совпадении  
  //NVIC_SetPriority(TIM1_CC_IRQn,5);                                      // приоритет прерывания
  RCC->APB2ENR|= RCC_APB2ENR_TIM1EN;                                     // тактирование таймера Т1 
  TIM1->CR1 &= ~(1<<0);                                                  // стоп таймер Т1  
  TIM1->DIER |= TIM_DIER_CC1IE;                                          // разрешить прерывание по сравнению Т1
  TIM1->DIER |= TIM_DIER_UIE;                                            // разрешить прерывания по обновлению счета Т1                                   
  TIM1->PSC = 16800-1;                                                   // предделитель таймера Т1 ( за 1 сек считает до 10000)
  TIM1->ARR = 10000-1;                                                   // кол-во тиков до перезагрузки Т1 
  TIM1->CCR1 =8000;                                                      // кол-во тиков до совпадения 1
  TIM1->CCR2 =8000;                                                      // кол-во тиков до совпадения 2
  TIM1->CCR3 =8000;                                                      // кол-во тиков до совпадения 3
  TIM1->CCR4 =8000;                                                      // кол-во тиков до совпадения 4
  TIM1->CR1 |=(1<<0);                                                    // старт таймера Т1
}

void second_blink ()
        {  
          if(TIM1->SR&TIM_SR_UIF)
             {
              TIM1->SR &= ~TIM_SR_UIF;
              GPIOB->BSRR=1<<16;             
             }
          if(TIM1->SR&TIM_SR_CC1IF)
             {
              TIM1->SR &= ~TIM_SR_CC1IF;
              GPIOB->BSRR=1<<0;
             }
        }
void TIM1_CC_IRQHandler(void)
        {
          TIM1->SR |= TIM_SR_CC2IF;
          GPIOB->BSRR = 1<<7;
        }

int main() 
{	
  setup_core_STM32F405();
	timer_1 ();
  while(1)
   {
	    second_blink();
   }
}

