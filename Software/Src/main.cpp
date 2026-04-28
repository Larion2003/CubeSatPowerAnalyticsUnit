#include "main.hpp"
#include "stm32l431xx.h"
#include <cstddef>

// Anonymous namespace
namespace {
    // ADC Channel configuration
    constexpr size_t ADC1_CHANNEL = 5;
    // System clock frequency (4 MHz)
    constexpr size_t freq_sys = 4000000;
    // Target baud rate for USART1
    constexpr size_t target_baudrate = 9600;

   /**
     * @brief Initializes the Independent Watchdog (IWDG).
     * @details Sets a timeout of approximately 1 second using the LSI clock.
     * * Calculations based on RM0394 Reference Manual:
     * - LSI Frequency (f_LSI) ≈ 32 kHz
     * - Prescaler (PR) = 64 (Register value 0x04)
     * - Reload Value (RLR) = 500
     * * Formula: Timeout = (Prescaler * RLR) / f_LSI
     * Timeout = (64 * 500) / 32000 = 1.0 seconds.
     * * Once started, the IWDG cannot be stopped except by a system reset.
     */
    void init_watchdog() {
        // Write access key to IWDG_KR to unlock PR and RLR registers
        // Refer to RM0394, IWDG_KR register description
        IWDG->KR = 0x5555;

        // Set the Prescaler to 64
        // 32 kHz / 64 = 500 Hz (Each counter tick is 2ms)
        IWDG->PR = 0x04;

        // Set the Reload Value (RLR)
        // 500 ticks * 2ms = 1000ms (1 second timeout)
        IWDG->RLR = 500;

        // Reload the counter with the RLR value (Refresh)
        // Writing 0xAAAA also protects the registers again
        IWDG->KR = 0xAAAA;

        // Start the watchdog counter
        // After this, the software must write 0xAAAA to KR regularly
        IWDG->KR = 0xCCCC;
    }

    /**
     * @brief Calculates the USART Baud Rate Register (BRR) value.
     * @details Refer to STM32L4 Reference Manual (RM0394), page 743.
     * Formula: BRR = fck / BaudRate
     */
    constexpr size_t calculate_brr(size_t freq, size_t baud) {
        return freq / baud;
    }

    // Compile-time calculation of the baud rate prescaler
    constexpr size_t usart1_brr_value = calculate_brr(freq_sys, target_baudrate);

    /**
     * @brief Compile-time validation of the baud rate error.
     * @details Ensures that the integer division rounding error stays within 5%.
     */
    static_assert((freq_sys / target_baudrate) * target_baudrate > (freq_sys * 0.95),
        "Baud rate configuration error exceeds 5% threshold!"
    );

}


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
    // and preserve SWD pins (PA13 = JTMS/SWDIO, PA14 = JTCK/SWCLK)

    // Optimized GPIOA MODER configuration:

    // Clear mode bits for USART1 (PA9, PA10) and SWD (PA13, PA14) pins
    GPIOA->MODER &= ~(GPIO_MODER_MODE9 |
                     GPIO_MODER_MODE10 |
                     GPIO_MODER_MODE13 |
                     GPIO_MODER_MODE14);

    // Set pins to Alternate Function mode (Mode 10)
    // Note: PA13 and PA14 must remain in AF mode to maintain debug (SWD) access
    GPIOA->MODER |= (GPIO_MODER_MODE9_1 |
                    GPIO_MODER_MODE10_1 |
                    GPIO_MODER_MODE13_1 |
                    GPIO_MODER_MODE14_1);

    // Configure Alternate Function 7 for USART1 (PA9, PA10)
    // AFR[1] handles pins 8 to 15 (AFRH)
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10);
    GPIOA->AFR[1] |= (7 << GPIO_AFRH_AFSEL9_Pos) | (7 << GPIO_AFRH_AFSEL10_Pos);

    // Note: SWD pins (PA13, PA14) use AF0 by default,
    // so we don't strictly need to rewrite their AFR bits,
    // but preserving their MODER bits is crucial.


    // Configure USART1 BaudRate (9600 bps)

    // Formula: USARTDIV = fck / BaudRate
    // MSI clock is 4 MHz (reset value) ==> 4,000,000 / 9600 = 416.66
    USART1->BRR = usart1_brr_value;


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

    // TODO: Enable Independent Watchdog (IWDG) before final production release.
    // init_watchdog();

	while(true){
        // code
    }

    return 0;
}
