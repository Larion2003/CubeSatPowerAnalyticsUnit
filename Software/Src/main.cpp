#include "main.hpp"
#include "stm32l431xx.h"
#include <cstdio>
#include <cstddef>
#include <cstdint>
#include <cstring>

// ==================================================================================================
// TISAT-PROTOCOL SLAVE MODULE: PWR (Power Measurement)
// ==================================================================================================
// Frame Structure (Max 64 bytes total):
//   [direction char(1)] [module id(3)] [payload(<=55, ends with '%')] [checksum(2 hex chars)] [\r\n]
//
//   '$' -> Inbound command from Master (Pico W) to Slave
//   '#' -> Outbound response from Slave to Master
//
//   Checksum = 8-bit sum (mod 256) of bytes from direction char up to '%',
//              formatted as 2 uppercase ASCII hex digits.
//
// Commands Handled:
//   - "ping"    -> "pong"              : Liveness verification (Polled every 5s)
//   - "GETDATA" -> "U:..,I:..,P:.."    : Telemetry request     (Polled every 3s)
// ==================================================================================================

namespace {

    // -------------------------------- CONSTANTS --------------------------------

    constexpr const size_t   ADC1_CHANNEL    = 5;          // PA0 current shunt input channel
    constexpr const uint32_t ADC_CH0_VREFINT = 0;          // Internal reference voltage channel
    constexpr const float    SHUNT_RESISTOR  = 0.05f;      // 50 mOhm shunt resistance
    constexpr const size_t   FREQ_SYS        = 4000000;    // 4 MHz MSI system clock
    constexpr const size_t   TARGET_BAUDRATE = 9600;
    constexpr const uint8_t  MSG_BUFFER_SIZE = 64;         // Max frame buffer length

    // Module Identification
    constexpr const char MODULE_ID[3] = {'P', 'W', 'R'};
    constexpr const char DIR_FROM_MASTER = '$';
    constexpr const char DIR_TO_MASTER   = '#';

    constexpr const char* CMD_PING     = "ping";
    constexpr const char* CMD_GET_DATA = "GETDATA";
    constexpr const char* RESP_PONG    = "pong";

    // Factory calibration address for internal voltage reference (1.212V typical)
    const uint16_t* VREF_INT_CAL_ADDR = reinterpret_cast<uint16_t*>(0x1FFF75AA);

    // Baud Rate Generator Register Value: 4,000,000 / 9600 = 416
    constexpr size_t USART1_BRR = FREQ_SYS / TARGET_BAUDRATE;

    // -------------------------------- BUFFERS --------------------------------

    char     tx_buffer[MSG_BUFFER_SIZE];
    volatile size_t tx_index  = 0;
    volatile size_t tx_length = 0;

    char     rx_buffer[MSG_BUFFER_SIZE];
    volatile size_t rx_index = 0;
    volatile bool   rx_ready = false;

    // -------------------------------- TELEMETRY CACHE --------------------------------

    volatile float shunt_voltage = 0.0f;
    volatile float shunt_current = 0.0f;
    volatile float shunt_power   = 0.0f;

    // -------------------------------- PROTOCOL HELPERS --------------------------------

    /**
     * @brief Computes Tisat 8-bit sum checksum (mod 256) up to '%' marker.
     */
    uint8_t calc_checksum(const char* frame, size_t percent_index) {
        uint8_t sum = 0;
        for (size_t i = 0; i <= percent_index; i++) {
            sum += static_cast<uint8_t>(frame[i]);
        }
        return sum;
    }

    char nibble_to_hex(uint8_t n) {
        const char LUT[] = "0123456789ABCDEF";
        return LUT[n & 0x0F];
    }

    uint8_t hex_to_nibble(char c) {
        if (c >= '0' && c <= '9') return static_cast<uint8_t>(c - '0');
        if (c >= 'A' && c <= 'F') return static_cast<uint8_t>(c - 'A' + 10);
        if (c >= 'a' && c <= 'f') return static_cast<uint8_t>(c - 'a' + 10);
        return 0;
    }

    int find_percent(const char* buf, size_t len) {
        for (size_t i = 0; i < len; i++) {
            if (buf[i] == '%') return static_cast<int>(i);
        }
        return -1;
    }

    /**
     * @brief Constructs Tisat response frame and initiates non-blocking TX interrupt stream.
     * Output format: '#' + "PWR" + payload + '%' + checksum(2 hex) + "\r\n"
     *
     * In single-wire half-duplex mode, transmitted bytes echo back onto the RX line.
     * RXNEIE is temporarily disabled during TX and re-enabled in the ISR once transmission completes.
     */
    void send_response(const char* payload) {
        size_t idx = 0;

        tx_buffer[idx++] = DIR_TO_MASTER;
        tx_buffer[idx++] = MODULE_ID[0];
        tx_buffer[idx++] = MODULE_ID[1];
        tx_buffer[idx++] = MODULE_ID[2];

        for (const char* p = payload; *p != '\0' && idx < (MSG_BUFFER_SIZE - 5); p++) {
            tx_buffer[idx++] = *p;
        }

        const size_t percent_index = idx;
        tx_buffer[idx++] = '%';

        const uint8_t chk = calc_checksum(tx_buffer, percent_index);
        tx_buffer[idx++] = nibble_to_hex(chk >> 4);
        tx_buffer[idx++] = nibble_to_hex(chk & 0x0F);

        tx_buffer[idx++] = '\r';
        tx_buffer[idx++] = '\n';

        // Disable RX interrupt to prevent parsing localized line transmission echoes
        USART1->CR1 &= ~USART_CR1_RXNEIE;

        tx_length = idx;
        tx_index  = 0;

        // Trigger TXE interrupt stream
        USART1->CR1 |= USART_CR1_TXEIE;
    }

    // -------------------------------- ADC HELPER --------------------------------

    uint32_t read_adc(uint32_t channel) {
        ADC1->SQR1 = (channel << ADC_SQR1_SQ1_Pos);
        ADC1->CR  |= ADC_CR_ADSTART;
        while (!(ADC1->ISR & ADC_ISR_EOC));
        return ADC1->DR;
    }

} // anonymous namespace

// -------------------------------- DELAY HELPER --------------------------------

static void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    // System clock @ 4 MHz: 1 us = 4 clock cycles
    while ((DWT->CYCCNT - start) < (us * 4));
}

// ==================================================================================================
// SystemInit — Peripherals Initialization Routine
// ==================================================================================================
extern "C" void SystemInit() {

    // Enable DWT Cycle Counter for delay timing
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    // Enable Peripheral Clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;   // GPIOA Clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // USART1 Clock
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;     // ADC1 Clock

    // ==========================================================================
    // USART1 Configuration — Half-Duplex Single-Wire (9600 Baud)
    // Pin: PA9 (Single shared TX/RX line)
    // ==========================================================================

    // PA9 & PA10 -> Alternate Function Mode (AF7 = USART1)
    // Preserve PA13 & PA14 debug pin configurations
    GPIOA->MODER &= ~(GPIO_MODER_MODE9  |
                      GPIO_MODER_MODE10 |
                      GPIO_MODER_MODE13 |
                      GPIO_MODER_MODE14);
    GPIOA->MODER |=  (GPIO_MODER_MODE9_1  |
                      GPIO_MODER_MODE10_1 |
                      GPIO_MODER_MODE13_1 |
                      GPIO_MODER_MODE14_1);

    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10);
    GPIOA->AFR[1] |=  (7u << GPIO_AFRH_AFSEL9_Pos) |
                      (7u << GPIO_AFRH_AFSEL10_Pos);

    USART1->BRR = USART1_BRR;

    // Enable Hardware TX Output Inversion (TXINV) for active line interface circuit matching
    USART1->CR2 |= USART_CR2_TXINV;

    // Enable Half-Duplex Single-Wire Mode
    USART1->CR3 |= USART_CR3_HDSEL;

    // Enable Transmitter, Receiver, Peripheral, and RX Not-Empty Interrupts
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;

    // PA9 Output Type: OPEN-DRAIN
    // Configured after UE=1 to prevent driver overrides. Open-drain configuration
    // allows the bus line to float HIGH via external pull-up resistor when idle.
    GPIOA->OTYPER |= (1u << 9u);

    NVIC_EnableIRQ(USART1_IRQn);

    // ==========================================================================
    // ADC1 Configuration — PA0 Shunt Input & VREFINT Channel
    // ==========================================================================

    // PA0 -> Analog Mode
    GPIOA->MODER &= ~GPIO_MODER_MODE0;
    GPIOA->MODER |=  GPIO_MODER_MODE0_0 | GPIO_MODER_MODE0_1;

    // Clock Selection: Synchronous AHB / 1
    ADC1_COMMON->CCR |= (1u << ADC_CCR_CKMODE_Pos);

    // Disable Deep Power Down, Enable Voltage Regulator
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |=  ADC_CR_ADVREGEN;
    delay_us(20);

    // Execute Calibration Routine
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);

    // Sequence Length = 1 Conversion
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR1 |=  (ADC1_CHANNEL << ADC_SQR1_SQ1_Pos);

    // Enable ADC Peripheral
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
}

// ==================================================================================================
// USART1 Interrupt Service Routine
// ==================================================================================================
extern "C" void USART1_IRQHandler() {

    // Transmit Branch (TXE)
    if ((USART1->ISR & USART_ISR_TXE) && (USART1->CR1 & USART_CR1_TXEIE)) {
        if (tx_index < tx_length) {
            USART1->TDR = static_cast<uint8_t>(tx_buffer[tx_index]);
            tx_index += 1;
        } else {
            // Transmission Complete — Disable TXE Interrupt
            USART1->CR1 &= ~USART_CR1_TXEIE;

            // Clear residual line echo bytes and overrun flags resulting from half-duplex operation
            (void)USART1->RDR;
            if (USART1->ISR & USART_ISR_ORE) {
                USART1->ICR = USART_ICR_ORECF;
            }

            // Flush partial frame state and re-enable RX interrupt
            rx_index = 0;
            rx_ready = false;
            USART1->CR1 |= USART_CR1_RXNEIE;
        }
    }

    // Receive Branch (RXNE)
    if ((USART1->ISR & USART_ISR_RXNE) && (USART1->CR1 & USART_CR1_RXNEIE)) {

        // Clear error condition flags (Framing, Noise, Overrun) silently
        if (USART1->ISR & (USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) {
            USART1->ICR = USART_ICR_FECF | USART_ICR_NCF | USART_ICR_ORECF;
            (void)USART1->RDR;
            return;
        }

        char c = static_cast<char>(USART1->RDR);

        if (c == DIR_FROM_MASTER) {
            // Master Start Frame Character Received
            rx_index = 0;
            rx_buffer[rx_index] = c;
            rx_index += 1;
        } else if (rx_index > 0) {
            if (rx_index < (MSG_BUFFER_SIZE - 1)) {
                rx_buffer[rx_index] = c;
                rx_index += 1;

                if (c == '\n') {
                    // Frame Termination Received
                    rx_buffer[rx_index] = '\0';
                    rx_ready = true;
                    // Pause RX interrupts until processing completes in main()
                    USART1->CR1 &= ~USART_CR1_RXNEIE;
                }
            } else {
                // Overflow Protection Reset
                rx_index = 0;
            }
        }
    }
}

// ==================================================================================================
// Main Execution Loop
// ==================================================================================================
int main() {
    // Enable Internal Reference Channel
    ADC1_COMMON->CCR |= ADC_CCR_VREFEN;
    delay_us(20);

    uint32_t vref_raw = 0;
    uint32_t shunt_raw = 0;
    float    vdda = 0.0f;

    char payload_buf[MSG_BUFFER_SIZE];

    while (true) {

        // Refresh Telemetry Measurements
        vref_raw = read_adc(ADC_CH0_VREFINT);
        vdda     = 3.0f * (static_cast<float>(*VREF_INT_CAL_ADDR) /
                           static_cast<float>(vref_raw));

        shunt_raw     = read_adc(ADC1_CHANNEL);
        shunt_voltage = (static_cast<float>(shunt_raw) / 4095.0f) * vdda;
        shunt_current = shunt_voltage / SHUNT_RESISTOR;
        shunt_power   = shunt_voltage * shunt_current;

        // Process Incoming Command
        if (rx_ready) {
            const int pct = find_percent(rx_buffer, rx_index);

            bool ok = (rx_buffer[0] == DIR_FROM_MASTER) &&
                      (pct >= 4) &&
                      (static_cast<size_t>(pct) + 2 < rx_index);

            if (ok) {
                const uint8_t rx_chk =
                    static_cast<uint8_t>(
                        (hex_to_nibble(rx_buffer[pct + 1]) << 4) |
                         hex_to_nibble(rx_buffer[pct + 2]));
                ok = (rx_chk == calc_checksum(rx_buffer, static_cast<size_t>(pct)));
            }

            if (ok) {
                const bool for_us =
                    (rx_buffer[1] == MODULE_ID[0]) &&
                    (rx_buffer[2] == MODULE_ID[1]) &&
                    (rx_buffer[3] == MODULE_ID[2]);

                if (for_us) {
                    const size_t payload_len = static_cast<size_t>(pct) - 4;
                    memcpy(payload_buf, &rx_buffer[4], payload_len);
                    payload_buf[payload_len] = '\0';

                    if (strcmp(payload_buf, CMD_PING) == 0) {
                        send_response(RESP_PONG);

                    } else if (strcmp(payload_buf, CMD_GET_DATA) == 0) {
                        snprintf(payload_buf, MSG_BUFFER_SIZE,
                                 "U:%.4fV,I:%.4fA,P:%.4fW",
                                 static_cast<double>(shunt_voltage),
                                 static_cast<double>(shunt_current),
                                 static_cast<double>(shunt_power));
                        send_response(payload_buf);
                    }
                }
            }

            // Command Processed — Reset Buffer & Re-arm RX Interrupts
            rx_index = 0;
            rx_ready = false;
            USART1->CR1 |= USART_CR1_RXNEIE;
        }
    }

    return 0;
}
