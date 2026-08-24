import time
import board
import busio
import digitalio
import gc
import traceback

# ==============================================================================
# HARDWARE INITIALIZATION
# ==============================================================================

try:
    led = digitalio.DigitalInOut(board.LED)
except AttributeError:
    import microcontroller
    led = digitalio.DigitalInOut(microcontroller.pin.GPIO25)
led.direction = digitalio.Direction.OUTPUT

print("--- Pico W Tisat-compatible MASTER (OBC simulator) started ---")

# ==============================================================================
# TISAT PROTOCOL SPECIFICATION
# ==============================================================================
# Master role (OBC simulator) driving single-wire half-duplex communication.
# Target slave module: STM32L431KC power-measurement board (ID: "PWR").
#
# Frame layout:
#   [direction(1)] [module id(3)] [payload] [%] [checksum(2 hex)] [\r\n]
#
#   '$' = Sent BY the master (outbound) -> Slave receives this
#   '#' = Sent BY the slave  (inbound)  -> Master receives this
#
#   checksum = Sum of all bytes from direction char up to and including '%',
#              mod 256, formatted as 2 uppercase hex characters.
#
# BIT-BANGING INVERSION MECHANICS:
# The single-wire half-duplex bus utilizes an external active pull-up circuit.
# Signal inversion occurs across this physical interface. To present standard
# UART logic levels to the slave's PA9 RX pin, the master pre-inverts software
# bit-banging transmissions:
#   - Bus Idle State   : LOW  (Converts to HIGH at slave)
#   - Start Bit        : HIGH (Converts to LOW at slave)
#   - Data Bits        : Inverted logic (0 -> HIGH, 1 -> LOW)
#   - Stop Bit         : LOW  (Converts to HIGH at slave)

MODULE_ID          = "PWR"
DIR_FROM_MASTER    = "$"
DIR_TO_MASTER      = "#"
CMD_PING           = "ping"
CMD_GET_DATA       = "GETDATA"
RESP_PONG          = "pong"
PING_INTERVAL_S    = 5.0
DATA_INTERVAL_S    = 3.0
RESPONSE_TIMEOUT_S = 1.0


# ==============================================================================
# TISAT FRAME BUILDING & PARSING
# ==============================================================================

def calc_checksum(frame, percent_index):
    """
    Computes 8-bit sum (mod 256) of every byte from direction character 
    up to and including the '%' delimiter.
    """
    checksum = 0
    for i in range(percent_index + 1):
        checksum = (checksum + ord(frame[i])) % 256
    return checksum


def build_frame(payload):
    """
    Constructs a valid outbound Tisat command frame for the target module:
        '$PWR' + payload + '%' + checksum(2 hex chars) + '\r\n'
    """
    body = DIR_FROM_MASTER + MODULE_ID + payload
    percent_index = len(body)
    body_with_terminator = body + "%"
    checksum = calc_checksum(body_with_terminator, percent_index)
    return body_with_terminator + "{:02X}".format(checksum) + "\r\n"


def parse_frame(raw_line):
    """
    Validates and extracts payload from inbound slave response frames.
    Returns extracted payload string if valid, or None if validation fails.
    """
    if len(raw_line) < 4 or raw_line[0] != DIR_TO_MASTER:
        return None
    if raw_line[1:4] != MODULE_ID:
        return None
    percent_index = raw_line.find("%")
    if percent_index < 4 or percent_index + 2 >= len(raw_line):
        return None
    payload = raw_line[4:percent_index]
    received_checksum_str = raw_line[percent_index + 1: percent_index + 3]
    try:
        received_checksum = int(received_checksum_str, 16)
    except ValueError:
        return None
    calculated_checksum = calc_checksum(raw_line, percent_index)
    if received_checksum != calculated_checksum:
        print("[WARN] Checksum mismatch! Got {}, expected {:02X}".format(
            received_checksum_str, calculated_checksum))
        return None
    return payload


# ==============================================================================
# HARDWARE UART RECEIVER (9600 Baud on GP1)
# ==============================================================================

def init_rx_uart():
    """
    Initializes hardware UART receiver on GP1.
    Receives non-inverted signal logic following external circuit reversal.
    """
    return busio.UART(None, board.GP1, baudrate=9600, timeout=1.0)


uart = init_rx_uart()
print("UART Receiver Initialized (Listening on GP1)")


# ==============================================================================
# SOFTWARE UART TRANSMITTER (Inverted Polarity on GP1)
# ==============================================================================

def send_frame_software(message):
    """
    Transmits frames via software bit-banging at 9600 baud using inverted polarity
    to account for single-wire bus interface inversion.
    """
    tx_pin = digitalio.DigitalInOut(board.GP1)
    tx_pin.direction = digitalio.Direction.OUTPUT

    # Inverted Idle: LOW
    tx_pin.value = False
    time.sleep(0.005)

    bit_delay = 1.0 / 9600.0  # ~104.17 us bit timing

    for byte in message.encode("utf-8"):
        # Inverted Start bit: HIGH
        tx_pin.value = True
        time.sleep(bit_delay)

        # 8 Data bits (LSB first, inverted)
        for i in range(8):
            bit = (byte >> i) & 1
            tx_pin.value = (bit == 0)
            time.sleep(bit_delay)

        # Inverted Stop bit: LOW
        tx_pin.value = False
        time.sleep(bit_delay)

    # Line stabilization before releasing pin
    tx_pin.value = False
    time.sleep(0.005)
    tx_pin.deinit()


# ==============================================================================
# HALF-DUPLEX COMMAND TRANSMISSION
# ==============================================================================

def send_command(payload):
    """
    Builds and transmits frame over GP1, safely handling GPIO direction 
    switching between TX bit-banging and RX hardware UART modes.
    """
    global uart

    frame = build_frame(payload)

    # Release hardware receiver to allow bit-banging output
    uart.deinit()
    uart = None
    gc.collect()

    time.sleep(0.015)  # Bus guard time

    print("[TX to STM32]: {}".format(frame.strip()))
    send_frame_software(frame)

    time.sleep(0.005)  # Bus settling time

    # Re-enable hardware receiver for slave response
    uart = init_rx_uart()
    time.sleep(0.01)
    uart.reset_input_buffer()


# ==============================================================================
# RESPONSE RECEIVING
# ==============================================================================

def receive_response(timeout_s=RESPONSE_TIMEOUT_S):
    """
    Polls hardware UART for incoming '\n'-terminated frame.
    Strips noise characters and identifies frame starting with '#'.
    """
    deadline = time.monotonic() + timeout_s
    rx_buffer = bytearray()

    while time.monotonic() < deadline:
        if uart and uart.in_waiting:
            char = uart.read(1)
            if char:
                rx_buffer.extend(char)
                if char == b'\n':
                    clean = bytearray(
                        b for b in rx_buffer if 32 <= b <= 126 or b in (10, 13)
                    )
                    raw_line = clean.decode().strip()
                    idx = raw_line.find(DIR_TO_MASTER)
                    if idx != -1:
                        return raw_line[idx:]
                    return None

    return None  # Response timeout


# ==============================================================================
# MAIN POLLING LOOP
# ==============================================================================

led_last_toggle = time.monotonic()
led_state       = False
last_ping_time  = time.monotonic()
last_data_time  = time.monotonic()

while True:
    now = time.monotonic()

    # Activity Indicator
    if now - led_last_toggle >= 1.5:
        led_state = not led_state
        led.value = led_state
        led_last_toggle = now

    try:
        # Periodic Liveness Check (Every 5s)
        if now - last_ping_time >= PING_INTERVAL_S:
            last_ping_time = now
            send_command(CMD_PING)
            response = receive_response()
            if response is None:
                print("[{}] No response to ping (module offline/unresponsive)".format(MODULE_ID))
            else:
                payload = parse_frame(response)
                if payload == RESP_PONG:
                    print("[{}] Pong received - module alive".format(MODULE_ID))
                else:
                    print("[{}] Unexpected ping reply: {}".format(MODULE_ID, response))

        # Periodic Telemetry Request (Every 3s)
        if now - last_data_time >= DATA_INTERVAL_S:
            last_data_time = now
            send_command(CMD_GET_DATA)
            response = receive_response()
            if response is None:
                print("[{}] No response to GETDATA".format(MODULE_ID))
            else:
                payload = parse_frame(response)
                if payload is not None:
                    print("[{}] Telemetry: {}".format(MODULE_ID, payload))
                else:
                    print("[{}] Invalid telemetry reply: {}".format(MODULE_ID, response))

    except Exception as e:
        print("\n!!! LOOP ERROR !!!")
        traceback.print_exception(None, e, e.__traceback__)
        print("----------------------------\n")
        try:
            if uart:
                uart.deinit()
        except Exception:
            pass
        uart = None
        gc.collect()
        uart = init_rx_uart()
