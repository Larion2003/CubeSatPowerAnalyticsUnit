#include "main.hpp"
#include "stm32l431xx.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

// Forward declarations: these are defined further down in this file, but
// send_frame() (inside the anonymous namespace below) needs to call them
// before their actual definitions appear.
char uart_getc();
void uart_putc(char c);
void uart_puts(const char* str);
void wait_free_bus();

// ==================================================================================================
// PWR MODULE — TiSAT-compatible slave communication protocol
// ==================================================================================================
// Frame format (max 64 bytes total), matching the shared TiSAT protocol:
//   [direction: 1 char] [module: 3 chars] [payload] [%] [checksum: 2 hex chars] [\r\n]
//
//   '$' -> command sent by the master, addressed to a module
//   '#' -> reply sent by a module back to the master
//
// Checksum: 8-bit sum (wrapping at 256) of every byte from the direction char
// up to and including '%', written as 2 uppercase hex digits.
//
// Commands handled by this module:
//   "ping"    -> "pong"                  : liveness check
//   "GETDATA" -> "I:..,U:..,W:.."        : current, voltage and power readout
// ==================================================================================================

namespace {

    // -------------------------------- CONSTANTS --------------------------------

    constexpr const size_t   ADC1_CHANNEL    = 5;          // PA0 current shunt input channel
    constexpr const float    SHUNT_RESISTOR  = 0.05f;      // 50 mOhm shunt resistance
    constexpr const size_t   FREQ_SYS        = 4000000;    // 4 MHz MSI system clock
    constexpr const size_t   TARGET_BAUDRATE = 9600;
    constexpr const uint8_t  MSG_BUFFER_SIZE = 64;          // Max frame length, matches the protocol spec

    // Factory calibration address for internal voltage reference (1.212V typical)
    const uint16_t* VREF_INT_CAL_ADDR = reinterpret_cast<uint16_t*>(0x1FFF75AA);

    // Baud Rate Generator Register Value: 4,000,000 / 9600 = 416
    constexpr size_t USART1_BRR = FREQ_SYS / TARGET_BAUDRATE;

    // -------------------------------- PROTOCOL IDENTITY --------------------------------

    constexpr const char MODULE_ID[3]    = {'P', 'W', 'R'};
    constexpr const char DIR_FROM_MASTER = '$';   // Command addressed to this module
    constexpr const char DIR_TO_MASTER   = '#';   // Reply sent back to the master

    constexpr const char* CMD_PING     = "ping";
    constexpr const char* CMD_GET_DATA = "GETDATA";
    constexpr const char* RESP_PONG    = "pong";

    // -------------------------------- TELEMETRY CACHE --------------------------------
    // Reported as-is in GETDATA replies. Real ADC measurement comes later.

    volatile float shunt_voltage = 0.0f;
    volatile float shunt_current = 0.0f;
    volatile float shunt_power   = 0.0f;

    // -------------------------------- FRAME BUFFERS --------------------------------

    char    rx_buffer[MSG_BUFFER_SIZE];   // Accumulates one incoming frame, byte by byte
    uint8_t rx_index = 0;

    char    tx_buffer[MSG_BUFFER_SIZE];   // Holds one outgoing frame while it is built and sent

    // -------------------------------- PROTOCOL HELPERS --------------------------------

    // Sums every byte from index 0 up to and including index 'percent_index' (the '%' byte),
    // wrapping at 256 via uint8_t overflow. Matches the shared protocol's checksum rule.
    uint8_t calc_checksum(const char* frame, size_t percent_index) {
        uint8_t sum = 0;
        for (size_t i = 0; i <= percent_index; i++) {
            sum += static_cast<uint8_t>(frame[i]);
        }
        return sum;
    }

    // Converts a 4-bit value (0-15) into its uppercase ASCII hex digit.
    // Used to encode the 1-byte checksum as 2 printable characters.
    char nibble_to_hex(uint8_t n) {
        const char LUT[] = "0123456789ABCDEF";
        return LUT[n & 0x0F];
    }

    // Converts a single ASCII hex digit ('0'-'9', 'A'-'F', 'a'-'f') back into
    // its 4-bit value. Used to decode the checksum received from the master.
    uint8_t hex_to_nibble(char c) {
        if (c >= '0' && c <= '9') return static_cast<uint8_t>(c - '0');
        if (c >= 'A' && c <= 'F') return static_cast<uint8_t>(c - 'A' + 10);
        if (c >= 'a' && c <= 'f') return static_cast<uint8_t>(c - 'a' + 10);
        return 0;
    }

    // Returns the index of the payload-terminating '%', or -1 if not found in [0, len).
    int find_percent(const char* buf, size_t len) {
        for (size_t i = 0; i < len; i++) {
            if (buf[i] == '%') return static_cast<int>(i);
        }
        return -1;
    }

    // Builds a full reply frame ('#PWR' + payload + '%' + checksum + "\r\n") and transmits it.
    //
    // On this single-wire bus, every transmitted byte is reflected back onto RX, so RE is
    // disabled for the duration of the send and only re-enabled once the line is confirmed
    // free (TC flag) and any leftover RXNE has been flushed. This is the same technique that
    // already proved stable for the plain "pong\n" reply.
    void send_frame(const char* payload) {
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

        USART1->CR1 &= ~USART_CR1_RE;   // Receiver off: keep our own echo out of RDR

        for (size_t i = 0; i < idx; i++) {
            uart_putc(tx_buffer[i]);
        }
        wait_free_bus();                 // Block until the last byte has fully left the line

        (void)USART1->RDR;               // Clear any leftover RXNE flag before RX is re-enabled

        USART1->CR1 |= USART_CR1_RE;    // Receiver back on: ready for the next real command
    }

    // Validates and dispatches one complete, "\n"-terminated frame from rx_buffer.
    void process_frame(const char* buf, size_t len) {
        const int pct = find_percent(buf, len);

        // A valid frame needs at least a direction char, 3 module chars, and a '%',
        // plus room for the 2 checksum digits after it.
        if (pct < 4 || static_cast<size_t>(pct) + 2 >= len) {
            return;
        }

        if (buf[0] != DIR_FROM_MASTER) {
            return;   // Not a command addressed to a module (e.g. our own echo slipping through)
        }

        const uint8_t rx_chk = static_cast<uint8_t>(
            (hex_to_nibble(buf[pct + 1]) << 4) | hex_to_nibble(buf[pct + 2]));
        if (rx_chk != calc_checksum(buf, static_cast<size_t>(pct))) {
            return;   // Corrupted frame, silently dropped
        }

        const bool for_us = (buf[1] == MODULE_ID[0]) &&
                             (buf[2] == MODULE_ID[1]) &&
                             (buf[3] == MODULE_ID[2]);
        if (!for_us) {
            return;   // Addressed to a different module
        }

        char payload[MSG_BUFFER_SIZE];
        const size_t payload_len = static_cast<size_t>(pct) - 4;
        memcpy(payload, &buf[4], payload_len);
        payload[payload_len] = '\0';

        if (strcmp(payload, CMD_PING) == 0) {
            send_frame(RESP_PONG);

        } else if (strcmp(payload, CMD_GET_DATA) == 0) {
            char data_payload[MSG_BUFFER_SIZE];
            snprintf(data_payload, sizeof(data_payload), "I:%.1f,U:%.1f,W:%.1f",
                      static_cast<double>(shunt_current),
                      static_cast<double>(shunt_voltage),
                      static_cast<double>(shunt_power));
            send_frame(data_payload);
        }
    }

} // anonymous namespace

// ==================================================================================================
// SystemInit — Peripherals Initialization Routine
// ==================================================================================================
extern "C" void SystemInit() {

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
// Continuously reads one byte at a time from the master and accumulates it into rx_buffer.
// Once a full line ('\n') has arrived, the buffered bytes make up one complete frame, which is
// handed off to process_frame() for validation and dispatch. rx_index is then reset, ready to
// accumulate the next frame.
int main() {

    char c;
    while (true)
    {
        c = uart_getc();

        // Safety: a frame that somehow exceeds the buffer without a terminator
        // (e.g. line noise) is discarded so the parser can resync on the next one.
        if (rx_index >= MSG_BUFFER_SIZE) {
            rx_index = 0;
        }

        rx_buffer[rx_index++] = c;

        if (c == '\n')
        {
            process_frame(rx_buffer, rx_index);
            rx_index = 0;
        }
    }

    return 0;
}
