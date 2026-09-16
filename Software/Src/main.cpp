#include "main.hpp"
#include "stm32l431xx.h"
#include <cstddef>
#include <cstdint>

namespace {
    // -------------------------------- CONSTANTS --------------------------------

    constexpr const size_t   ADC1_CHANNEL    = 5;          // PA0 current shunt input channel
    constexpr const float    SHUNT_RESISTOR  = 0.05f;      // 50 mOhm shunt resistance
    constexpr const size_t   FREQ_SYS        = 4000000;    // 4 MHz MSI system clock
    constexpr const size_t   TARGET_BAUDRATE = 9600;

	// -------------------------------- TELEMETRY CACHE --------------------------------
    // Populated once ADC-based shunt measurement is implemented.

    volatile float shunt_voltage = 0.0f;
    volatile float shunt_current = 0.0f;
    volatile float shunt_power   = 0.0f;
    
    // Factory calibration address for internal voltage reference (1.212V typical)
    const uint16_t* VREF_INT_CAL_ADDR = reinterpret_cast<uint16_t*>(0x1FFF75AA);

    // Baud Rate Generator Register Value: 4,000,000 / 9600 = 416
    constexpr size_t USART1_BRR = FREQ_SYS / TARGET_BAUDRATE;
    
} // anonymous namespace

// ==================================================================================================
// SystemInit — Peripherals Initialization Routine
// ==================================================================================================
extern "C" void SystemInit() 
{

    // Enables the DWT (Data Watchpoint and Trace) cycle counter.
    // CYCCNT increments once per core clock cycle and is used as a
    // free-running timer for cycle-accurate delay measurements.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    // Enables the peripheral clocks required by this module.
    // A peripheral's registers cannot be accessed until its clock is enabled.
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;   // GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // USART1 clock
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;     // ADC1 clock

    // ==========================================================================
    // USART1 Configuration — Half-Duplex Single-Wire (9600 Baud)
    // Pin: PA9 (Single shared TX/RX line)
    // ==========================================================================

    // Sets PA9 (USART1_TX) and PA10 (USART1_RX) to Alternate Function mode.
    // PA13/PA14 (SWDIO/SWCLK) are intentionally left untouched here — after
    // reset, they are already in Alternate Function mode by default, so the
    // debugger stays connected without any firmware configuration needed.
    GPIOA->MODER &= ~(GPIO_MODER_MODE9  | GPIO_MODER_MODE10);
    GPIOA->MODER |=  (GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);

    // Selects AF7 (USART1) as the alternate function routed to PA9 and PA10.
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10);
    GPIOA->AFR[1] |=  (7u << GPIO_AFRH_AFSEL9_Pos) |
    (7u << GPIO_AFRH_AFSEL10_Pos);

    // Baud rate register: USART1_BRR = system clock / target baud rate.
    // At a 4 MHz system clock this yields 9600 baud.
    USART1->BRR = USART1_BRR;

    // Inverts the TX line logic level. The board routes PA9 through an external
    // hardware inverter, so this compensates for it and restores a
    // non-inverted signal on the physical bus.
    USART1->CR2 |= USART_CR2_TXINV;

    // Disables the overrun error flag (ORE). Without this, an unread byte
    // being overwritten by a new one would block further reception until
    // the flag is cleared in software.
    USART1->CR3 = USART_CR3_OVRDIS;

    // Enables the transmitter (TE), the receiver (RE), and the USART
    // peripheral itself (UE). Without UE, none of the above settings take effect.
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

// Blocks until a byte has been fully received (RXNE flag set), then reads
// and returns it. Reading RDR also clears the RXNE flag.
char uart_getc()
{
    while (!(USART1->ISR & USART_ISR_RXNE)) {}
    return USART1->RDR;
}

// Blocks until the transmit data register is empty (TXE flag set), then
// loads the next byte into TDR to start shifting it out on the line.
void uart_putc(char c)
{
    while (!(USART1->ISR & USART_ISR_TXE)) {}

    USART1->TDR = static_cast<uint8_t>(c);
}

// Sends a null-terminated string one byte at a time via uart_putc(),
// stopping at the terminating '\0'.
void uart_puts(const char* str)
{
    while (*str) {
        uart_putc(*str++);
    }
}

// Blocks until the last byte has been fully shifted out onto the line
// (TC flag set), meaning transmission is physically complete — not just
// that TDR is empty (TXE), but that the stop bit has also left the pin.
void wait_free_bus(){while (!(USART1->ISR & USART_ISR_TC)) {}}

// ==================================================================================================
// Main Execution Loop
// ==================================================================================================
int main() {

	// ping\n - pong\n test
    char c;
    while (true)
    {
        c = uart_getc();

        if (c == '\n')
        {
            // Receiver disabled during transmission: on this single-wire bus,
            // transmitted bytes are reflected back onto RX, so RE stays off
            // to keep the echo out of RDR.
            USART1->CR1 &= ~USART_CR1_RE;

            uart_puts("pong\n");
            wait_free_bus();     // Blocks until "pong\n" has fully left the line (TC flag set)
            (void)USART1->RDR;   // Clears any leftover RXNE flag before RX is re-enabled

            // Receiver re-enabled only after transmission is fully complete,
            // so only a genuine "ping" from the master will be detected next.
            USART1->CR1 |= USART_CR1_RE;
        }
    }

    return 0;
}
