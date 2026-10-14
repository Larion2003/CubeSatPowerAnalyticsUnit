#include "main.hpp"
#include "stm32l431xx.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

// ==================================================================================================
// PWR MODULE — power-measurement slave of the TiSAT protocol
// ==================================================================================================
// Measures the supply current through a shunt resistor (INA199 amplifier -> 12-bit ADC) and reports
// current, voltage and power to the master (Raspberry Pi Pico W) over a shared single-wire UART bus.
//
// Frame format (max 64 bytes), shared by all TiSAT modules:
//   [direction: 1 char] [module: 3 chars] [payload] [%] [checksum: 2 hex chars] [\r\n]
//
//   '$' -> command sent by the master, addressed to a module
//   '#' -> reply sent by a module back to the master
//
// Checksum: 8-bit sum (wrapping at 256) of every byte from the direction char up to and
// including '%', written as 2 uppercase hex digits.
//
// Commands handled by this module:
//   "ping"    -> "pong"                         : liveness check
//   "GETDATA" -> "I:<x> mA,U:<x> mV,W:<x> mW"   : current, voltage and power readout
// ==================================================================================================

namespace {

    // -------------------------------- MEASUREMENT CHAIN --------------------------------
    // Load current -> shunt resistor -> INA199 (x50) -> ADC1 channel 5 (PA0).
    // The supply voltage (VDDA) is measured separately through the ADC's internal VREFINT channel.

    constexpr uint32_t ADC1_CHANNEL     = 5;          // PA0: INA199 output (amplified shunt voltage)
    constexpr uint32_t ADC_CH0_VREFINT  = 0;          // ADC channel 0: internal reference voltage (VREFINT)
    constexpr float    SHUNT_RESISTOR   = 0.0522865f; // Ohm: 52.2865 mOhm, measured with a 4-point method
    constexpr float    INA199_GAIN      = 50.0f;      // V/V: gain of the INA199A1 current-sense amplifier
    constexpr float    ADC_FULL_SCALE   = 4095.0f;    // Largest 12-bit ADC code (2^12 - 1), equals VDDA
    constexpr float    VREFINT_CAL_VDDA = 3.0f;       // V: supply at which ST measured VREFINT_CAL in the factory

    // Address of VREFINT_CAL: the ADC code of VREFINT that ST measured at VDDA = 3.0 V (stored in flash).
    const uint16_t* const VREF_INT_CAL_ADDR = reinterpret_cast<const uint16_t*>(0x1FFF75AA);

    // -------------------------------- CURRENT CALIBRATION --------------------------------
    // The ideal chain  I_raw = V_adc / (INA199_GAIN * SHUNT_RESISTOR)  does not give the true
    // current. Without correction the firmware reads about 17.5 % too high, plus a small negative
    // offset of about 0.5 mA, because of the gain/offset tolerances of the amplifier and the ADC.
    // The error follows a straight line over the whole range, so a linear correction is enough.
    //
    // How the constants were obtained:
    //  1. Run the firmware with CURRENT_CAL_GAIN = 1 and CURRENT_CAL_OFFSET = 0 (no correction)
    //     and with SHUNT_RESISTOR as above.
    //  2. Apply known reference currents and record the uncorrected value I_raw. Each point is the
    //     mean of 5 readings, and each reading is itself the mean of 64 ADC conversions.
    //  3. Fit a straight line by linear regression:   I_raw = GAIN * I_ref + OFFSET
    //  4. At run time the line is inverted:           I = (I_raw - OFFSET) / GAIN
    //
    // GAIN is the slope (dimensionless), OFFSET the intercept (in A).
    // Cross-check: an independent regression on the final 38-point dataset below gives
    // GAIN = 1.1787 and OFFSET = -0.52 mA, i.e. the slope agrees within 0.05 % and the intercept
    // within 0.07 mA (a fifth of an ADC step), so the values below were kept.
    //
    // Verification (38 reference points, 1-200 mA, mean of 5 readings each; 0 mA reads exactly 0):
    //   output vs. reference:  slope 1.0004, intercept -0.05 mA, R^2 = 0.999997
    //   absolute error:        mean -0.02 mA, mean |e| 0.092 mA, RMS 0.112 mA,
    //                          worst 0.194 mA (at 200 mA), i.e. 0.1 % of the 200 mA range
    //   relative error:        <= 0.29 % from 50 mA up, <= 0.92 % from 20 mA up,
    //                          <= 1.37 % from 10 mA up, worst 8.68 % at 1 mA
    //                          (the absolute error stays ~0.1 mA at every current, so the
    //                          percentage grows as the current gets smaller)
    constexpr float CURRENT_CAL_GAIN   = 1.17819f;      // Slope of the calibration line
    constexpr float CURRENT_CAL_OFFSET = -0.00045384f;  // Intercept of the calibration line, in A

    // The line is only valid where it was fitted. Extrapolated to I_raw = 0 it would output
    // -OFFSET/GAIN = 0.39 mA instead of 0. Below this raw value the output is therefore forced
    // to exactly 0. 0.5 mA is ~1.6 ADC steps (one step ~0.31 mA of current) and stays above the
    // no-load readings seen in all tests. A raw value of 0.5 mA corresponds to a true current of
    // (0.5 + 0.454) / 1.178 = 0.81 mA, so currents below ~0.8 mA are reported as 0.
    constexpr float CURRENT_MIN_VALID  = 0.0005f;       // A, compared with the uncorrected I_raw

    // Every ADC reading is the mean of this many conversions, which cuts random noise by
    // sqrt(64) = 8x. Cost: ~21 ms per GETDATA (2 channels x 64 x 163 us, see adc_init()).
    constexpr uint8_t ADC_AVERAGING_SAMPLES = 64;

    // -------------------------------- CLOCK, UART AND PROTOCOL --------------------------------

    constexpr uint32_t FREQ_SYS        = 4000000;     // Hz: default 4 MHz MSI system clock
    constexpr uint32_t TARGET_BAUDRATE = 9600;
    constexpr uint32_t USART1_BRR      = FREQ_SYS / TARGET_BAUDRATE;   // 4,000,000 / 9600 = 416
    constexpr uint8_t  MSG_BUFFER_SIZE = 64;          // Max frame length, matches the protocol spec

    constexpr char MODULE_ID[3]    = {'P', 'W', 'R'};
    constexpr char DIR_FROM_MASTER = '$';             // Command addressed to a module
    constexpr char DIR_TO_MASTER   = '#';             // Reply sent back to the master

    constexpr const char* CMD_PING     = "ping";
    constexpr const char* CMD_GET_DATA = "GETDATA";
    constexpr const char* RESP_PONG    = "pong";

    // One complete measurement, in SI units.
    struct Measurement {
        float voltage;   // V: supply voltage (VDDA), also the voltage of the monitored 3.3 V rail
        float current;   // A: current through the shunt
        float power;     // W: voltage * current
    };

    // -------------------------------- RECEIVE BUFFER --------------------------------
    // Filled byte by byte by main(); a complete frame ends with '\n' and is passed to process_frame().

    char    rx_buffer[MSG_BUFFER_SIZE];   // Accumulates the incoming frame
    uint8_t rx_index = 0;                 // Number of bytes currently in rx_buffer

    // -------------------------------- UART PRIMITIVES --------------------------------

    // Blocks until a byte has been received (RXNE flag), then returns it.
    // Reading RDR also clears the RXNE flag.
    char uart_getc() {
        while (!(USART1->ISR & USART_ISR_RXNE)) {}
        return static_cast<char>(USART1->RDR);
    }

    // Blocks until the transmit data register is empty (TXE flag), then loads the next byte
    // into TDR, which starts shifting it out on the line.
    void uart_putc(char c) {
        while (!(USART1->ISR & USART_ISR_TXE)) {}
        USART1->TDR = static_cast<uint8_t>(c);
    }

    // Blocks until the last byte has fully left the pin (TC flag). Stronger than TXE: TDR may be
    // empty while the final byte is still being shifted out.
    void wait_free_bus() {
        while (!(USART1->ISR & USART_ISR_TC)) {}
    }

    // -------------------------------- ADC HELPERS --------------------------------

    // Busy-waits for about 'us' microseconds using the DWT cycle counter started in SystemInit().
    // Only used for the short ADC regulator start-up delay, too brief to justify a hardware timer.
    void delay_us(uint32_t us) {
        const uint32_t start  = DWT->CYCCNT;
        const uint32_t cycles = us * (FREQ_SYS / 1000000);
        while (DWT->CYCCNT - start < cycles) {}
    }

    // Brings up ADC1 once, before the first conversion: leaves deep-power-down, enables the
    // internal voltage regulator, calibrates the ADC and switches it on (power-up sequence from
    // the reference manual's ADC chapter).
    //
    // PA0 needs no GPIO setup: pins reset into analog mode, which is what the ADC input needs.
    void adc_init() {
        ADC1->CR &= ~ADC_CR_DEEPPWD;     // Wake up from deep-power-down (the reset state)
        ADC1->CR |= ADC_CR_ADVREGEN;     // Enable the ADC's internal voltage regulator
        delay_us(20);                    // Regulator start-up time (T_ADCVREG_STUP)

        // Clock ADC1 straight from HCLK, no prescaler: the simplest choice, valid because the
        // system runs from the default, undivided 4 MHz MSI clock.
        ADC1_COMMON->CCR = (ADC1_COMMON->CCR & ~ADC_CCR_CKMODE) | ADC_CCR_CKMODE_0;

        // Connect the internal reference voltage so it can be read like any other channel
        // (channel 0, see ADC_CH0_VREFINT).
        ADC1_COMMON->CCR |= ADC_CCR_VREFEN;

        ADC1->CR |= ADC_CR_ADCAL;             // Start single-ended calibration
        while (ADC1->CR & ADC_CR_ADCAL) {}    // Wait until it completes

        // Longest sample time (640.5 ADC clock cycles) for the shunt channel and VREFINT: accuracy
        // matters far more than speed, and the 1 kOhm filter resistor (R2) in front of the shunt
        // channel needs time to charge the ADC's internal sampling capacitor.
        // One conversion = 640.5 + 12.5 cycles at 4 MHz = ~163 us.
        ADC1->SMPR1 |= ADC_SMPR1_SMP0 | ADC_SMPR1_SMP5;

        ADC1->CR |= ADC_CR_ADEN;                  // Enable the ADC
        while (!(ADC1->ISR & ADC_ISR_ADRDY)) {}   // Wait until it reports ready
    }

    // Runs one single conversion of 'channel' and returns the raw 12-bit code (0..4095).
    uint16_t adc_read_channel(uint32_t channel) {
        ADC1->SQR1 = (channel << ADC_SQR1_SQ1_Pos);  // Sequence of length 1: convert only 'channel'
        ADC1->CR |= ADC_CR_ADSTART;                  // Start the conversion

        while (!(ADC1->ISR & ADC_ISR_EOC)) {}        // Wait for "end of conversion"
        return static_cast<uint16_t>(ADC1->DR);      // Reading DR also clears EOC
    }

    // Returns the mean of 'samples' conversions of 'channel' (oversampling to reduce noise).
    // The integer division truncates (mean bias about -0.5 code, ~-0.15 mA of raw current). The
    // calibration and the accuracy figures above were obtained with this behaviour, so changing it
    // (e.g. to rounding) would require a new calibration.
    uint16_t adc_read_channel_averaged(uint32_t channel, uint8_t samples) {
        uint32_t sum = 0;
        for (uint8_t i = 0; i < samples; i++) {
            sum += adc_read_channel(channel);
        }
        return static_cast<uint16_t>(sum / samples);
    }

    // -------------------------------- MEASUREMENT --------------------------------

    // Takes one fresh measurement of supply voltage, current and power.
    //
    //  1. VDDA is not assumed to be exactly 3.3 V. The ADC code of the fixed internal reference
    //     (VREFINT) is inversely proportional to VDDA, and ST stored its code at 3.0 V in flash
    //     (VREFINT_CAL), so  VDDA = 3.0 V * VREFINT_CAL / code.  This is far more accurate than a
    //     nominal value, and it is also the ADC's full-scale voltage used in step 2.
    //  2. The shunt-channel code becomes a voltage:  code / 4095 * VDDA  (INA199 output).
    //  3. The INA199 multiplies the shunt's voltage drop by its fixed gain, so dividing by the gain
    //     recovers the drop, and Ohm's law (I = V / R) gives the uncorrected current I_raw.
    //  4. I_raw is corrected with the calibration line, or forced to 0 below CURRENT_MIN_VALID.
    //  5. Power is P = VDDA * I. VDDA doubles as the measured voltage "U" because the shunt sits on
    //     the board's own 3.3 V rail (see T1/V_In on the schematic).
    Measurement read_measurement() {
        const uint16_t vrefint_raw = adc_read_channel_averaged(ADC_CH0_VREFINT, ADC_AVERAGING_SAMPLES);
        const float vdda = VREFINT_CAL_VDDA *
            (static_cast<float>(*VREF_INT_CAL_ADDR) / static_cast<float>(vrefint_raw));

        const uint16_t shunt_raw = adc_read_channel_averaged(ADC1_CHANNEL, ADC_AVERAGING_SAMPLES);
        const float ina_out_voltage = (static_cast<float>(shunt_raw) / ADC_FULL_SCALE) * vdda;

        const float shunt_drop_voltage = ina_out_voltage / INA199_GAIN;
        const float current_raw        = shunt_drop_voltage / SHUNT_RESISTOR;

        Measurement m;
        m.voltage = vdda;
        if (current_raw < CURRENT_MIN_VALID) {
            m.current = 0.0f;
        } else {
            m.current = (current_raw - CURRENT_CAL_OFFSET) / CURRENT_CAL_GAIN;
        }
        m.power = m.voltage * m.current;
        return m;
    }

    // -------------------------------- PROTOCOL HELPERS --------------------------------

    // Sums every byte from index 0 up to and including 'percent_index' (the '%' byte), wrapping
    // at 256 through uint8_t overflow. Matches the shared protocol's checksum rule.
    uint8_t calc_checksum(const char* frame, size_t percent_index) {
        uint8_t sum = 0;
        for (size_t i = 0; i <= percent_index; i++) {
            sum += static_cast<uint8_t>(frame[i]);
        }
        return sum;
    }

    // Converts a 4-bit value (0-15) to its uppercase ASCII hex digit.
    // Used to write the 1-byte checksum as 2 printable characters.
    char nibble_to_hex(uint8_t n) {
        static constexpr char LUT[] = "0123456789ABCDEF";
        return LUT[n & 0x0F];
    }

    // Converts one ASCII hex digit ('0'-'9', 'A'-'F', 'a'-'f') to its 4-bit value (0 if invalid).
    // Used to read the checksum received from the master.
    uint8_t hex_to_nibble(char c) {
        if (c >= '0' && c <= '9') return static_cast<uint8_t>(c - '0');
        if (c >= 'A' && c <= 'F') return static_cast<uint8_t>(c - 'A' + 10);
        if (c >= 'a' && c <= 'f') return static_cast<uint8_t>(c - 'a' + 10);
        return 0;
    }

    // Returns the index of the first '%' in buf[0..len), or -1 if there is none.
    int find_percent(const char* buf, size_t len) {
        for (size_t i = 0; i < len; i++) {
            if (buf[i] == '%') return static_cast<int>(i);
        }
        return -1;
    }

    // Builds a full reply frame ('#PWR' + payload + '%' + checksum + "\r\n") and transmits it.
    //
    // On this single-wire bus every transmitted byte is also received on RX (echo). The receiver
    // is therefore switched off while sending, and switched on again only after the last byte has
    // left the line (TC flag) and any leftover RXNE flag has been cleared.
    void send_frame(const char* payload) {
        char tx_buffer[MSG_BUFFER_SIZE];
        size_t idx = 0;

        tx_buffer[idx++] = DIR_TO_MASTER;
        tx_buffer[idx++] = MODULE_ID[0];
        tx_buffer[idx++] = MODULE_ID[1];
        tx_buffer[idx++] = MODULE_ID[2];

        // Keep 5 bytes free for '%', the 2 checksum digits and "\r\n".
        for (const char* p = payload; *p != '\0' && idx < (MSG_BUFFER_SIZE - 5); p++) {
            tx_buffer[idx++] = *p;
        }

        const size_t percent_index = idx;
        tx_buffer[idx++] = '%';

        const uint8_t checksum = calc_checksum(tx_buffer, percent_index);
        tx_buffer[idx++] = nibble_to_hex(checksum >> 4);
        tx_buffer[idx++] = nibble_to_hex(checksum & 0x0F);

        tx_buffer[idx++] = '\r';
        tx_buffer[idx++] = '\n';

        USART1->CR1 &= ~USART_CR1_RE;   // Receiver off: keep our own echo out of RDR

        for (size_t i = 0; i < idx; i++) {
            uart_putc(tx_buffer[i]);
        }
        wait_free_bus();                // Wait until the last byte has fully left the line

        (void)USART1->RDR;              // Clear any leftover RXNE flag before RX is re-enabled

        USART1->CR1 |= USART_CR1_RE;    // Receiver on: ready for the next command
    }

    // Validates one complete, "\n"-terminated frame and answers it if it is a known command for
    // this module. Invalid, corrupted or foreign frames are dropped silently (no reply).
    void process_frame(const char* buf, size_t len) {
        const int pct = find_percent(buf, len);

        // A valid frame has a direction char and 3 module chars before the '%', and 2 checksum
        // digits after it.
        if (pct < 4 || static_cast<size_t>(pct) + 2 >= len) {
            return;
        }

        if (buf[0] != DIR_FROM_MASTER) {
            return;   // Not a command from the master (e.g. our own echo slipping through)
        }

        const uint8_t rx_checksum = static_cast<uint8_t>(
            (hex_to_nibble(buf[pct + 1]) << 4) | hex_to_nibble(buf[pct + 2]));
        if (rx_checksum != calc_checksum(buf, static_cast<size_t>(pct))) {
            return;   // Corrupted frame
        }

        if (!(buf[1] == MODULE_ID[0] && buf[2] == MODULE_ID[1] && buf[3] == MODULE_ID[2])) {
			return;   // Addressed to a different module
		}

        // Copy the payload (between the module ID and the '%') into a terminated string.
        char payload[MSG_BUFFER_SIZE];
        const size_t payload_len = static_cast<size_t>(pct) - 4;
        memcpy(payload, &buf[4], payload_len);
        payload[payload_len] = '\0';

        if (strcmp(payload, CMD_PING) == 0) {
            send_frame(RESP_PONG);

        } else if (strcmp(payload, CMD_GET_DATA) == 0) {
            const Measurement m = read_measurement();   // Fresh reading right before replying

            // Reported in mA, mV and mW, e.g. "I:100.1168 mA,U:3318.5483 mV,W:332.2424 mW".
            char data_payload[MSG_BUFFER_SIZE];
            snprintf(data_payload, sizeof(data_payload), "I:%.4f mA,U:%.4f mV,W:%.4f mW",
                     static_cast<double>(m.current * 1000.0f),
                     static_cast<double>(m.voltage * 1000.0f),
                     static_cast<double>(m.power * 1000.0f));
            send_frame(data_payload);
        }
    }

} // anonymous namespace

// ==================================================================================================
// SystemInit — peripheral initialisation (called by the startup code before main())
// ==================================================================================================
extern "C" void SystemInit() {

    // Enable the DWT (Data Watchpoint and Trace) cycle counter. CYCCNT increments once per core
    // clock cycle and serves as a free-running timer for delay_us().
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    // Enable the clocks of the peripherals used here; their registers are not accessible before.
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;   // GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // USART1
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;     // ADC1

    // ==========================================================================
    // USART1 — 9600 baud on one shared bus wire
    // PA9 = USART1_TX, PA10 = USART1_RX. The USART runs in normal full-duplex mode, but both pins
    // are connected to the same bus wire, so every transmitted byte is also received (see send_frame).
    // ==========================================================================

    // Set PA9 and PA10 to alternate-function mode. PA13/PA14 (SWDIO/SWCLK) are left untouched:
    // they are already in alternate-function mode after reset, which keeps the debugger connected.
    GPIOA->MODER &= ~(GPIO_MODER_MODE9  | GPIO_MODER_MODE10);
    GPIOA->MODER |=  (GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);

    // Select AF7 (USART1) as the alternate function of PA9 and PA10.
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10);
    GPIOA->AFR[1] |=  (7u << GPIO_AFRH_AFSEL9_Pos) |
                      (7u << GPIO_AFRH_AFSEL10_Pos);

    // Baud rate register: system clock / baud rate (4 MHz / 9600 = 416).
    USART1->BRR = USART1_BRR;

    // Invert the TX line. The board routes PA9 through an external hardware inverter; this
    // compensates for it and restores a non-inverted signal on the bus.
    USART1->CR2 |= USART_CR2_TXINV;

    // Disable overrun detection (OVRDIS): if a byte arrives while RDR is still unread, the new
    // byte simply overwrites it and no ORE error is raised, so the receiver never stalls.
    USART1->CR3 = USART_CR3_OVRDIS;

    // Enable the transmitter (TE), the receiver (RE) and the USART itself (UE). The settings
    // above only take effect once UE is set.
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

// ==================================================================================================
// Main Execution Loop
// ==================================================================================================
// Continuously reads one byte at a time from the master and accumulates it into rx_buffer.
// Once a full line ('\n') has arrived, the buffered bytes make up one complete frame, which is
// handed off to process_frame() for validation and dispatch. rx_index is then reset, ready to
// accumulate the next frame.
int main() {

    adc_init();

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
