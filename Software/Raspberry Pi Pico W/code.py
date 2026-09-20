# TiSAT protocol test master for the Raspberry Pi Pico W.
#
# Periodically sends "ping" and "GETDATA" commands to the PWR module running
# on the STM32L431KC over a shared single-wire UART bus, and prints both the
# sent command and the module's reply.

import board
import busio
import time

uart = busio.UART(board.GP0, board.GP1, baudrate=9600)

MODULE = "PWR"

MASTER_LABEL = "Raspberry Pi Pico W (Master)"
SLAVE_LABEL  = "STM32L431KC - PWR module (Slave)"
LABEL_WIDTH  = max(len(MASTER_LABEL), len(SLAVE_LABEL))

# Prints one log line with the arrow aligned to the same column, regardless
# of how long the source label ("Master" vs "Slave") is.
def log(label, message):
    print(f"{label:<{LABEL_WIDTH}} -> {message}")

# Computes the TiSAT protocol checksum: the 8-bit sum (wrapping at 256) of
# every character in 'body', which already includes the trailing '%'.
# Must match calc_checksum() on the STM32 side exactly.
def calc_checksum(body):
    checksum = 0
    for ch in body:
        checksum = (checksum + ord(ch)) & 0xFF
    return checksum

# Builds one full TiSAT frame addressed to MODULE: "$" + module + payload +
# "%" + 2 hex checksum digits + "\r\n".
def build_frame(payload):
    body = "$" + MODULE + payload + "%"
    checksum = calc_checksum(body)
    return f"{body}{checksum:02X}\r\n"

# Sends one command frame and returns the slave's reply line.
def send_command(payload):
    log(MASTER_LABEL, payload)

    frame = build_frame(payload)
    uart.write(frame.encode("ascii"))

    # The shared single-wire bus reflects every transmitted byte back onto our
    # own RX, so the first line read back is always our own echoed command.
    echo = uart.readline()

    # The slave's real reply follows as the next complete line.
    reply = uart.readline()
    return reply

# Extracts the payload from a "#PWR...%.." reply line, or None if the line
# is missing, empty, or not a well-formed reply.
def extract_payload(line):
    if not line:
        return None
    try:
        text = line.decode().strip()
    except UnicodeError:
        return None   # A shared bus occasionally picks up noise; skip this line
    if len(text) < 5 or text[0] != "#":
        return None
    percent_index = text.find("%")
    if percent_index < 4:
        return None
    return text[4:percent_index]

# Alternates between a liveness check and a telemetry request, once per second.
while True:
    reply = send_command("ping")
    log(SLAVE_LABEL, extract_payload(reply))
    time.sleep(0.5)

    reply = send_command("GETDATA")
    log(SLAVE_LABEL, extract_payload(reply))
    time.sleep(0.5)
