Conversation opened. 1 unread message.

Skip to content
Using Gmail with screen readers
1 of 3,167
adc with bare metal
Inbox

sivaprasad nakka <sivaprasadnakka838@gmail.com>
Attachments
9:13 PM (0 minutes ago)
to me


 One attachment
  •  Scanned by Gmail
#include "stm32f446xx.h"
#include <stdio.h>

volatile uint16_t adc_value;

/* crude delay */
void delay_ms(uint32_t ms)
{
    for(uint32_t i = 0; i < ms * 4000; i++);
}

/* UART transmit */
void uart_send(char *str)
{
    while (*str)
    {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *str++;
    }
}

/* GPIO PA5 LED */
void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~(3 << (5 * 2));
    GPIOA->MODER |=  (1 << (5 * 2));   // PA5 output
}

/* UART2 init (PA2 TX, PA3 RX) */
void uart2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2)));
    GPIOA->MODER |=  (2 << (2 * 2)) | (2 << (3 * 2));   // AF

    GPIOA->AFR[0] &= ~((0xF << (2 * 4)) | (0xF << (3 * 4)));
    GPIOA->AFR[0] |=  (7 << (2 * 4)) | (7 << (3 * 4));  // AF7

    USART2->BRR = 16000000 / 115200;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

/* TIM2 → TRGO */
void tim2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 16000 - 1;   // 1 kHz timer clock
    TIM2->ARR = 10 - 1;      // 100 Hz trigger rate

    TIM2->CR2 &= ~(7 << 4);
    TIM2->CR2 |=  (2 << 4);  // TRGO = update event

    TIM2->CR1 |= TIM_CR1_CEN;
}

/* ADC1 + DMA2 Stream0 */
void adc_dma_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* PA0 analog */
    GPIOA->MODER |= (3 << (0 * 2));

    /* ADC prescaler */
    ADC->CCR |= ADC_CCR_ADCPRE_0;   // PCLK2 / 4

    /* ---------- DMA CONFIG ---------- */
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN);

    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)&adc_value;
    DMA2_Stream0->NDTR = 1;

    DMA2_Stream0->CR =
        (0 << 25) |            // Channel 0
        DMA_SxCR_CIRC |        // Circular mode
        DMA_SxCR_PSIZE_0 |     // Peripheral size 16-bit
        DMA_SxCR_MSIZE_0 |     // Memory size 16-bit
        DMA_SxCR_PL_1;         // High priority

    DMA2_Stream0->CR |= DMA_SxCR_EN;

    /* ---------- ADC CONFIG ---------- */
    ADC1->CR1 = 0;

    ADC1->CR2 =
        ADC_CR2_DMA |          // Enable DMA
        ADC_CR2_DDS |          // DMA continuous
        (6 << 24) |            // EXTSEL = TIM2 TRGO
        (1 << 28);             // EXTEN = rising edge  ✅ FIX

    ADC1->SMPR2 |= (7 << 0);   // Long sample time
    ADC1->SQR3 = 0;            // Channel 0

    ADC1->CR2 |= ADC_CR2_ADON;     // Enable ADC
    ADC1->CR2 |= ADC_CR2_SWSTART;  // Arm ADC
}

int main(void)
{
    char buf[40];

    gpio_init();
    uart2_init();
    tim2_init();
    adc_dma_init();

    while (1)
    {
        sprintf(buf, "ADC: %d\r\n", adc_value);
        uart_send(buf);

        if (adc_value < 1500)
            GPIOA->ODR &= ~(1 << 5);
        else if (adc_value < 3000)
            GPIOA->ODR |= (1 << 5);
        else
        {
            GPIOA->ODR ^= (1 << 5);
            delay_ms(200);
        }

        delay_ms(100);
    }
}
main.c
Displaying main.c.
