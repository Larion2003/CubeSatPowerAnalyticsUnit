# Test master for the PWR module: sends ping/GETDATA over the shared
# single-wire UART bus and logs both sides of the conversation.

import board
import busio
import digitalio
import time

uart = busio.UART(board.GP0, board.GP1, baudrate=9600)

led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

MODULE = "PWR"

MASTER_LABEL = "Raspberry Pi Pico W (Master)"
SLAVE_LABEL  = "STM32L431KC - PWR module (Slave)"
LABEL_WIDTH  = max(len(MASTER_LABEL), len(SLAVE_LABEL))

# Lines up the arrow in the same column no matter how long the label is.
def log(label, message):
    print(f"{label:<{LABEL_WIDTH}} -> {message}")

# Same checksum rule as the STM32 side: sum of 'body' (already has the
# trailing '%'), wrapping at 256.
def calc_checksum(body):
    checksum = 0
    for ch in body:
        checksum = (checksum + ord(ch)) & 0xFF
    return checksum

# "$" + module + payload + "%" + 2 hex checksum digits + "\r\n"
def build_frame(payload):
    body = "$" + MODULE + payload + "%"
    checksum = calc_checksum(body)
    return f"{body}{checksum:02X}\r\n"

# Sends one command frame and returns the slave's reply line.
def send_command(payload):
    log(MASTER_LABEL, payload)

    frame = build_frame(payload)
    uart.write(frame.encode("ascii"))

    # We hear our own bytes back on this shared bus, so the first line is our own echo.
    echo = uart.readline()

    # The slave's real reply follows next.
    reply = uart.readline()
    return reply

# Pulls the payload out of a "#PWR...%.." reply, or None if it's missing/malformed.
def extract_payload(line):
    if not line:
        return None
    try:
        text = line.decode().strip()
    except UnicodeError:
        return None   # bus noise, skip this line
    if len(text) < 5 or text[0] != "#":
        return None
    percent_index = text.find("%")
    if percent_index < 4:
        return None
    return text[4:percent_index]

# Alternates ping/GETDATA once a second, blinking the onboard LED as a heartbeat.
while True:
    reply = send_command("ping")
    log(SLAVE_LABEL, extract_payload(reply))
    led.value = not led.value
    time.sleep(0.5)

    reply = send_command("GETDATA")
    log(SLAVE_LABEL, extract_payload(reply))
    led.value = not led.value
    time.sleep(0.5)
