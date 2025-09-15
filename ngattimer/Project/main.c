#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>

void GPIO_Config(void);
void TIM3_Config_Delay(void);
void TIM2_Config_IRQ_500ms(void);
void TIM2_EnableIRQ(bool en);
void delay_ms(uint32_t ms);
bool Button_WasPressed(void);

#define LED1_PIN    0   // PA0
#define LD2_PIN     5   // PA5
#define BUTTON_PIN  13  // PC13

static volatile bool ld2_irq_mode = false; 

int main(void)
{
    GPIO_Config();           
    TIM3_Config_Delay();     
    TIM2_Config_IRQ_500ms(); 
    TIM2_EnableIRQ(false);   

    while(1)
    {
        GPIOA->ODR &= ~(1 << LED1_PIN);           
        if (!ld2_irq_mode) GPIOA->ODR &= ~(1 << LD2_PIN); 
        delay_ms(500);

        /* ON 500 ms */
        GPIOA->ODR |=  (1 << LED1_PIN);           
        if (!ld2_irq_mode) GPIOA->ODR |=  (1 << LD2_PIN); 
        delay_ms(500);

        if (Button_WasPressed()) {
            ld2_irq_mode = !ld2_irq_mode;
            TIM2_EnableIRQ(ld2_irq_mode);
        }
    }
}

void GPIO_Config(void)
{
    RCC->APB2ENR |= (1 << 0)  
                  | (1 << 2)  
                  | (1 << 4);

    GPIOA->CRL &= ~(0xF << (LED1_PIN * 4));
    GPIOA->CRL |=  (0x2 << (LED1_PIN * 4));

    GPIOA->CRL &= ~(0xF << (LD2_PIN * 4));
    GPIOA->CRL |=  (0x2 << (LD2_PIN * 4));

    GPIOC->CRH &= ~(0xF << ((BUTTON_PIN - 8) * 4));
    GPIOC->CRH |=  (0x8 << ((BUTTON_PIN - 8) * 4)); // 1000b
    GPIOC->ODR |=  (1 << BUTTON_PIN);

    GPIOA->ODR &= ~(1 << LD2_PIN);
    GPIOA->ODR &= ~(1 << LED1_PIN);
}


void TIM3_Config_Delay(void)
{
    RCC->APB1ENR |= (1 << 1);    
    TIM3->PSC = 72 - 1;         
    TIM3->ARR = 0xFFFF;
    TIM3->CNT = 0;
    TIM3->CR1 |= (1 << 0);      
}

void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
    {
        TIM3->CNT = 0;
        while (TIM3->CNT < 1000); 
    }
}


void TIM2_Config_IRQ_500ms(void)
{
    RCC->APB1ENR |= (1 << 0); 

    TIM2->PSC = 7200 - 1;     
    TIM2->ARR = 5000 - 1;     
    TIM2->CNT = 0;

    TIM2->CR1 |= (1 << 0);    
}

void TIM2_EnableIRQ(bool en)
{
    if (en) {
        TIM2->SR  &= ~(1 << 0);   
        TIM2->CNT  = 0;           
        TIM2->DIER |=  (1 << 0);  
        NVIC_ClearPendingIRQ(TIM2_IRQn);
        NVIC_SetPriority(TIM2_IRQn, 2);
        NVIC_EnableIRQ(TIM2_IRQn);
    } else {
        TIM2->DIER &= ~(1 << 0);  
        NVIC_DisableIRQ(TIM2_IRQn);
    }
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & (1 << 0))   
    {
        TIM2->SR &= ~(1 << 0);         
        GPIOA->ODR ^= (1 << LD2_PIN);  
    }
}

bool Button_WasPressed(void)
{
    if ((GPIOC->IDR & (1 << BUTTON_PIN)) == 0) { 
        delay_ms(20);
        if ((GPIOC->IDR & (1 << BUTTON_PIN)) == 0) {
            while ((GPIOC->IDR & (1 << BUTTON_PIN)) == 0){};
            delay_ms(20);
            return true;
        }
    }
    return false;
}
