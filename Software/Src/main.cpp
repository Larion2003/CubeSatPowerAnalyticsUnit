#include "main.hpp"
#include "stm32l431xx.h"

#define ADC1_CHANNEL 5
#define USART1_DIV 417

/**
 * @brief  Provides a precise delay in microseconds.
 * @details Uses the processor's DWT cycle counter to wait.
 * At 4MHz, 1 us equals 4 CPU cycles.
 * @param  us: Delay time in microseconds.
 * @retval None
 */
static void delay_us(uint32_t us){
    uint32_t start_tick = DWT->CYCCNT;
    // Calculate total ticks: microseconds * (Frequency / 1,000,000)
    uint32_t delay_ticks = us * 4;

    while ((DWT->CYCCNT - start_tick) < delay_ticks);
}

/**
 * @brief  Performs low-level hardware initialization for the subsystem.
 * @retval None
 */
extern "C" void SystemInit(){

    // Enable DWT Cycle Counter hardware
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Reset counter
    DWT->CYCCNT = 0;
    // Start counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    ////////////////////////////////////////////////////////////////////
    // Reset and Clock Control - Enable peripheral clocks
    ////////////////////////////////////////////////////////////////////

    // Enable GPIOA clock (AHB2 bus)
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // Enable USART1 clock (APB2 bus)
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Enable ADC1 clock (AHB2 bus)
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;



    ////////////////////////////////////////////////////////////////////
    // USART1 CONFIG
    ////////////////////////////////////////////////////////////////////

    // Configure GPIOA pins for USART1 (PA9 = TX, PA10 = RX)

    // Clear mode bits for PA9 and PA10
    GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);

    // Set PA9 and PA10 to Alternate Function mode (Mode 10)
    GPIOA->MODER |= (GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);

    // Configure Alternate Function 7 (USART1_TX, USART1_RX) for PA9 and PA10
    // AFR[1] handles pins 8 to 15 (AFRH)
    // Clear bits
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10);
    // Set bits
    GPIOA->AFR[1] |= (7 << GPIO_AFRH_AFSEL9_Pos) | (7 << GPIO_AFRH_AFSEL10_Pos);


    // Configure USART1 BaudRate (9600 bps)

    // Formula: USARTDIV = fck / BaudRate
    // MSI clock is 4 MHz (reset value) ==> 4,000,000 / 9600 = 416.66
    USART1->BRR = USART1_DIV;


    // Configure USART1 Control Registers

    // Enable Half-Duplex mode (single wire communication)
    USART1->CR3 |= USART_CR3_HDSEL;
    // Enable Transmitter, Receiver and USART peripheral
    USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);



    ////////////////////////////////////////////////////////////////////
    // ADC1 CONFIG
    ////////////////////////////////////////////////////////////////////

    // Configure PA0 (= ADC1_IN) as Analog Mode

    // Clear mode bits for PA0
    GPIOA->MODER &= ~(GPIO_MODER_MODE0);
    // Set PA0 to Analog Mode (11)
    GPIOA->MODER |= (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE0_1);

    // Configure ADC Clock Source
    // Set ADC clock to synchronous with the AHB clock (divided by 1)
    ADC1_COMMON->CCR |= (1 << ADC_CCR_CKMODE_Pos);


    // Wake up ADC from deep power down

    // Exit deep power down mode
    ADC1->CR &= ~(ADC_CR_DEEPPWD);
    // Enable ADC internal voltage regulator
    ADC1->CR |= ADC_CR_ADVREGEN;
    // Wait a bit for the regulator to stabilize
    delay_us(20);

    // ADC Calibration

    // Start calibration
    ADC1->CR |= ADC_CR_ADCAL;
    // Wait until calibration is done
    while(ADC1->CR & ADC_CR_ADCAL);


    // Configure the sequence

    // Set sequence length to 1 (L=0 means 1 conversion) in SQR1 register
    ADC1->SQR1 &= ~(ADC_SQR1_L);
    // Set ADC channel (PA0)
    ADC1->SQR1 |= (ADC1_CHANNEL << ADC_SQR1_SQ1_Pos);


    // Enable ADC
    ADC1->CR |= ADC_CR_ADEN;
    // Wait until ADC is ready
    while(!(ADC1->ISR & ADC_ISR_ADRDY));

}

int main(){

    SystemInit();

	while(true){
        // code
    }

    return 0;
}
