#include "stm32f10x.h"

void GPIO_Config(void);
void TIM2_Config_Delay(void);
void delay_ms(uint32_t ms);

int main(void)
{
    GPIO_Config();
    TIM2_Config_Delay();

    while(1)
    {
        GPIOA->ODR &= ~(1 << 5); 
        delay_ms(500);

        GPIOA->ODR |= (1 << 5);
        delay_ms(500);
    }
}

void GPIO_Config(void)
{
    RCC->APB2ENR |= (1 << 2); 

    GPIOA->CRL &= ~(0xF << 20);
    GPIOA->CRL |=  (0x2 << 20);
    GPIOA->ODR |=  (1 << 5); 
}

void TIM2_Config_Delay(void)
{
    RCC->APB1ENR |= (1 << 0); 

    TIM2->PSC = 72 - 1;  
    TIM2->ARR = 0xFFFF;  
    TIM2->CR1 |= (1 << 0); 
}

void delay_ms(uint32_t ms)
{
    uint32_t i;
    for(i = 0; i < ms; i++)
    {
        TIM2->CNT = 0;                  
        while(TIM2->CNT < 1000);        
    }
}
