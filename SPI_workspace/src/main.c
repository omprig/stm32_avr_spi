#include "stm32f4xx.h"
#include <stdbool.h>

#define CONNECTION_CODE_MASTER 0xFF
#define CONNECTION_CODE_SLAVE 0xAA

volatile bool send_time = false;

void init_RTC();
void init_GPIO();
void init_clock();
void init_SPI2();
void set_SPI_connection();
void send_RTC_time();

void RTC_Alarm_IRQHandler(void)
{
    if(RTC->ISR & RTC_ISR_ALRAF)
    {  
    send_time = true;
    
    //toggle PD13
    GPIOD->ODR ^= (1 << 13);
    
    RTC->ISR &= ~(RTC_ISR_ALRAF);
    
    //reset pending bit
    EXTI->PR |= EXTI_PR_PR17;
    }
}

int main()
{
  init_clock();
  
  init_GPIO();
  
  init_SPI2();
  
    // wait for setting connection with slave
  set_SPI_connection();
  
  init_RTC();
    
  while(1)
  {
    if(send_time)
    {
        send_time = false;
        send_RTC_time();
    }
  }
  
  return 0;
}

void init_clock()
{
    // enable HSE and Clock Security System
    // HSE = 8MHz
    RCC->CR = (uint32_t)((uint32_t)RCC_CR_HSEON | (uint32_t)RCC_CR_CSSON);
    
    //wait for stable HSE
    while((RCC->CR & RCC_CR_HSERDY) == 0);
    
    // select HSE as SystemClock
    RCC->CFGR &= ~(0x00000003);
    RCC->CFGR |= 0x00000001;

    // wait 
    while((RCC->CFGR & 0x00000004) == 0);
    
    // configure AHB, APB1 and APB2 dividers
    // AHB frequency = HSE/2
    // APB1 frequency = AHB/2
    // APB2 frequency = AHB/2
    RCC->CFGR |= (uint32_t)(RCC_CFGR_HPRE_DIV2 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2);
}                       
    
void init_RTC()
{
    //--------------RCC init---------------
    
    //enable clock for PWR
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    //enable the access to RTC registers
    PWR->CR |= PWR_CR_DBP;
    
    //set divider for RTC_Clock 
    // RTC_Clock = HSE / 8 = 1MHz
    RCC->CFGR |= RCC_CFGR_RTCPRE_3;
    
    //enable RTC and select HSE/8 as a clock source, HSE_RTC 1MHz
    RCC->BDCR |= RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL;
    
    // write the key to disable write prtection on RTC registers
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    
    //enter to initialization mode
    RTC->ISR |= RTC_ISR_INIT;
    while((RTC->ISR & RTC_ISR_INITF) == 0);
    
    //set asynchonic and synchronic predividers
    // in order to get 1Hz PREDIVA = 124, PREDIVS = 7999
    RTC->PRER = 0x1F3F;
    RTC->PRER |= 0x7C << 16;
    
    // set time 21:53:48
    RTC->TR = 0x00215348;

    //clear INIT bit to leave initialization mode
    RTC->ISR &= ~RTC_ISR_INIT;    
    //set time format
    //RTC->CR |= RTC_CR_FMT
    
    // configure interrupt for RTC
    //set rising edge for triggering EXT17(RTC)
    EXTI->RTSR |= EXTI_RTSR_TR17;
    //unmask EXT17 interrupt
    EXTI->IMR |= EXTI_IMR_MR17;
    
    //enable event
    EXTI->EMR |= EXTI_EMR_MR17;
    
    //reset pending bit
    //EXTI->PR |= EXTI_PR_PR17;
    

    //enable interrupt via NVIC
    NVIC_EnableIRQ(RTC_Alarm_IRQn);
    
    //set priority
    NVIC_SetPriority(RTC_Alarm_IRQn, 0);
    
    //__iar_builtin_enable_interrupt();
    __enable_irq();

    RTC->ISR &= ~(RTC_ISR_ALRAF);
    
    //programming alarm A
    
    //disable alarm A and its interrupt
    
    RTC->CR &= ~(RTC_CR_ALRAE | RTC_CR_ALRAIE);
    
    //wait for ALARMWF
    while((RTC->ISR & RTC_ISR_ALRAWF) == 0);
    
    //configure alarm A
    RTC->ALRMAR |= RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK1;
    //alarm every second
    RTC->ALRMASSR = 0x00000000;
    
    //enable alarm A and correspding interrupt
    RTC->CR |= RTC_CR_ALRAE | RTC_CR_ALRAIE;
}
    
void init_SPI2()
{
    // configure appropriate pin for SPI
    //PB12 -- NSS
    //PB13 -- SCK
    //PB14 -- MISO
    //PB15 -- MOSI
    // reset PORTB
    RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOBRST;
    RCC->AHB1RSTR &= ~RCC_AHB1RSTR_GPIOBRST;
    
    // enable clocking for PORTB
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    
    //set alternative function mode 
    GPIOB->MODER |= GPIO_MODER_MODER15_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER12_1;
    
    // set push-pull for output type
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_15 | GPIO_OTYPER_OT_14 | GPIO_OTYPER_OT_13 | GPIO_OTYPER_OT_12);
    
    //set speed to high
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15 | GPIO_OSPEEDER_OSPEEDR14 | GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR12;
    
    //set pull-up for NSS and MISO
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR12_0 | GPIO_PUPDR_PUPDR14_0;
    
    //set corresponding alternative function, af05 - SPI mode
    GPIOB->AFR[1] |= (uint32_t)0x55550000;
   
    //reset SPI2
    RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST;
    RCC->APB1RSTR &= ~(RCC_APB1RSTR_SPI2RST);
    
    //enable SPI2
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    
    //configure SPI speed as 0.5 MHz
    SPI2->CR1 |= SPI_CR1_BR_0;
    
    // SPI mode 0, CPOL = 0, CPHA = 0
    SPI2->CR1 &= ~(SPI_CR1_CPHA | SPI_CR1_CPOL);
    
    // frame format - 8 bit
    SPI2->CR1 &= ~SPI_CR1_DFF;
    
    // transmit MSB first
   SPI2->CR1 &= ~SPI_CR1_LSBFIRST; 
   
   //enable NSS output in hardware mode
   SPI2->CR2 |= SPI_CR2_SSOE;
   
   //enable SPI in master mode
   SPI2->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE;
    
}

void init_GPIO()
{
    // reset PORTD
    RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIODRST;
    RCC->AHB1RSTR &= ~((uint32_t)RCC_AHB1RSTR_GPIODRST);
    
    // enable PORTD clock
    RCC->AHB1ENR |= (uint32_t)RCC_AHB1ENR_GPIODEN;
    
        // set output mode on PD13
    GPIOD->MODER = GPIO_MODER_MODER13_0;
    
    // set GPIOD speed to low
    GPIOD->OSPEEDR &= ~(3UL << 26);
    
    // set no pull-up or pull down
    GPIOD->PUPDR &= ~(3UL << 26);
    
    GPIOD->ODR = 1 << 13;
    
}

void set_SPI_connection()
{
    uint8_t code = 0x00;
    do{
        SPI2->DR = CONNECTION_CODE_MASTER;
        while (!(SPI2->SR & SPI_SR_RXNE));
        code = SPI2->DR;
    }while(code != CONNECTION_CODE_SLAVE);  
}

void send_RTC_time()
{
    uint8_t hours_BCD = (RTC->TR & RTC_TR_HT) >> 16;
    uint8_t minutes_BCD = (RTC->TR & RTC_TR_MNT) >> 8;
    uint8_t seconds_BCD = RTC->TR & RTC_TR_ST;
    
    hours_BCD |= (RTC->TR & RTC_TR_HU) >> 16;
    minutes_BCD |= (RTC->TR & RTC_TR_MNU) >> 8;
    seconds_BCD |= RTC->TR & RTC_TR_SU;
    
    //send hours
    SPI2->DR = hours_BCD;
    while(!(SPI2->SR & SPI_SR_TXE));
    
    //send minutes
    SPI2->DR = minutes_BCD;
    while(!(SPI2->SR & SPI_SR_TXE));
    
    //send seconds
    SPI2->DR = seconds_BCD;
    while(!(SPI2->SR & SPI_SR_TXE));
    
}