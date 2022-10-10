#include "stm32f0xx.h"
#include <math.h>   // for M_PI
#include "lcd.h"

int TIM15EN = 16;
int TIM7EN = 5;
int TIM6EN = 4;
int TIM2EN = 0;
uint32_t volume = 2048;
int add = 0;


extern const Picture background; // A 240x320 background image

void nano_wait(int);

//============================================================================
// enable_ports()
//============================================================================
void enable_ports(void)
{
    int en;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;



    //enable rcc
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //enable b outputs
    en = 0x3fffff;
    GPIOB->MODER |= en;
    en = 0x2aaaaa;
    GPIOB->MODER &= ~en;
    //enable c pins
    //en = 0xffff;
    //GPIOC->MODER |= en;
    //en = 0xaaff;
    //GPIOC->MODER &= ~en;
    //pull c inputs high
    en = 0xff;
    //GPIOC->PUPDR |= en;
    en = 0xaa;
    //GPIOC->PUPDR &= ~en;
    //set c otyper
    en = 0xf0;
    //GPIOC->OTYPER |= en;
    GPIOC->MODER &= ~GPIO_MODER_MODER0 & ~GPIO_MODER_MODER1 & ~GPIO_MODER_MODER2 & ~GPIO_MODER_MODER3;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR0 & ~GPIO_PUPDR_PUPDR1 & ~GPIO_PUPDR_PUPDR2 & ~GPIO_PUPDR_PUPDR3;
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR1_1 | GPIO_PUPDR_PUPDR2_1 | GPIO_PUPDR_PUPDR3_1;
}



//===========================================================================
// Part 4: Create an analog sine wave of a specified frequency
//===========================================================================
//void dialer(void);

// Parameters for the wavetable size and expected synthesis rate.
#define N 1000
#define RATE 20000
short int wavetable[N];
int step0 = 0;
int offset0 = 0;
int step1 = 0;
int offset1 = 0;


void init_wavetable(void)
{

    for(int i=0; i < N; i++)
        wavetable[i] = 32767 * sin(2 * M_PI * i / N);
}

//============================================================================
// set_freq()
//============================================================================
void set_freq(int chan, float f) {
    if (chan == 0) {
        if (f == 0.0) {
            step0 = 0;
            offset0 = 0;
        } else
            step0 = (f * N / RATE) * (1<<16);
    }
    if (chan == 1) {
        if (f == 0.0) {
            step1 = 0;
            offset1 = 0;
        } else
            step1 = (f * N / RATE) * (1<<16);
    }
}

//============================================================================
// setup_dac()
//============================================================================
void setup_dac(void)
{
    int en;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    en = 0x3d;
    DAC->CR |= en;
    en = 0x38;
    DAC->CR &= ~en;
}

//============================================================================
// Timer 6 ISR
//============================================================================

// Write the Timer 6 ISR here.  Be sure to give it the right name.

void TIM6_DAC_IRQHandler(void)  {
    int EN;
    EN = 1<<0;
    TIM6->SR &= ~EN;
    offset0 += step0;
    offset1 += step1;
    if (offset0 >= (N << 16))
        offset0 -= (N << 16);
    if (offset1 >= (N << 16))
        offset1 -= (N << 16);
    int samp = wavetable[offset0>>16] + wavetable[offset1>>16];
    samp = samp * volume;
    samp = samp >> 17;
    samp += 2048;
    DAC->DHR12R1 = samp;
}

//============================================================================
// init_tim6()
//============================================================================
void init_tim6(void)
{

    int EN;
    //enable rcc
    RCC->APB1ENR |= 1<<TIM6EN;
    EN = 2400 - 1;
    TIM6->ARR = EN;
    //set dier
    EN = 1<<0;
    TIM6->DIER |= EN;
    //enable counter
    TIM6->CR1 |= EN;
    //enable dac
    EN = 0x20;
    TIM6->CR2 |= EN;
    //set nvic
    EN = 1<<17;
    NVIC->ISER[0] = EN;

}

void init_tim2(void)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;

    TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

    TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);

    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

    TIM_Cmd(TIM2, ENABLE);

    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

    while(1)
    {}

}

void TIM2_IRQHandler(void)
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);


    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

    float I2CValue = TIM_GetCapture2(TIM2);

    if(I2CValue != 0)
    {
        int Frequency = 48000000 / I2CValue;

        if(Frequency == 100)
        {
            LCD_DrawString(20,141, BLACK, WHITE, "Help          ", 16, 0); // clear background
        }

        else if(Frequency == 110)
        {
            LCD_DrawString(20,141, BLACK, WHITE, "I am Okay     ", 16, 0); // clear background
        }

        else if(Frequency == 120)
        {
            LCD_DrawString(20,141, BLACK, WHITE, "Are you Okay? ", 16, 0); // clear background
        }

        else if(Frequency == 130)
        {
            LCD_DrawString(20,141, BLACK, WHITE, "Turn Left     ", 16, 0); // clear background
        }

        else if(Frequency == 140)
        {
            LCD_DrawString(20,141, BLACK, WHITE, "Turn Right    ", 16, 0); // clear background
        }

        else if(Frequency == 150)
        {
            LCD_DrawString(20,141, BLACK, WHITE, "Hello There   ", 16, 0); // clear background
        }

        else if(Frequency == 160)
        {
            LCD_DrawString(20,141, BLACK, WHITE, "Go Straight   ", 16, 0); // clear background
        }

        else if(Frequency == 180)
        {
            LCD_DrawString(20,141, BLACK, WHITE, "Where are you?", 16, 0); // clear background
        }

        else if(Frequency == 190)
        {
            LCD_DrawString(20,141, BLACK, WHITE, "Behind you    ", 16, 0); // clear background
        }


    }

}

void init_lcd_spi(void)
{
    //initialize gpiob
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER |= 0x30C30000;
    GPIOB->MODER &= ~0x20820000;
    GPIOB->ODR |= 0x4900;
    GPIOB->MODER |= 0xCC0;
    GPIOB->MODER &= ~0x440;
    GPIOB->AFR[0] &= ~0xF0F000;
    //initialize spi
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 &= ~1<<6; //turn of spe
    SPI1->CR1 &= ~0x38; //baud rate
    SPI1->CR1 |= 1<<2; //master mode
    SPI1->CR2 = 0x700; //8 bit word size
    SPI1->CR1 |= 1<<8; //ssm
    SPI1->CR1 |= 1<<9; //ssi
    SPI1->CR1 |= 1<<6; //turn on spe
}

void exti_setup(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC;
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PC;
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PC;
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI2;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PC;
    EXTI->RTSR |= EXTI_RTSR_TR0;
    EXTI->RTSR |= EXTI_RTSR_TR1;
    EXTI->RTSR |= EXTI_RTSR_TR2;
    EXTI->RTSR |= EXTI_RTSR_TR3;
    EXTI->IMR |= EXTI_IMR_MR0;
    EXTI->IMR |= EXTI_IMR_MR1;
    EXTI->IMR |= EXTI_IMR_MR2;
    EXTI->IMR |= EXTI_IMR_MR3;

    NVIC->ISER[0] = 1<<EXTI0_1_IRQn;
    NVIC->ISER[0] = 1<<EXTI2_3_IRQn;

}

void EXTI0_1_IRQHandler(void)
{
        if(EXTI->PR == 0x1) {
            EXTI->PR = EXTI_PR_PR0;
            set_freq(0,100 + add);
        }
        else if(EXTI->PR == 0x2) {
            EXTI->PR= EXTI_PR_PR1;
            set_freq(0, 130 + add);
        }
}

void EXTI2_3_IRQHandler(void)
{

    if(EXTI->PR == 0x4) {
        EXTI->PR = EXTI_PR_PR2;
        if(160 + add != 170) {
            set_freq(0, 160 + add);
        }
        else {
            set_freq(0, 190);
        }
    }
    else if(EXTI->PR  == 0x8) {
        EXTI->PR = EXTI_PR_PR3;
        add = add + 10;
        if(add == 30)
        {
            add = 0;
        }
    }

}






//============================================================================
// All the things you need to test your subroutines.
//============================================================================
int main(void)
{

    enable_ports();
    init_wavetable();
    setup_dac();
    init_tim6();
    exti_setup();
    LCD_Setup();
    init_tim2();

}
