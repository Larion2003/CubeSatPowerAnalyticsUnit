import time
import board
import busio
import digitalio

# Initialize the onboard LED safely across different Pico variants
try:
    led = digitalio.DigitalInOut(board.LED)
except AttributeError:
    import microcontroller
    led = digitalio.DigitalInOut(microcontroller.pin.GPIO25)
led.direction = digitalio.Direction.OUTPUT

print("--- Pico W Half-Duplex Receiver Started ---")

# CRITICAL FOR NO-EXTERNAL-RESISTOR:
# Configure GP5 with an internal pull-up resistor before enabling UART hardware.
# This prevents the open-drain bus from floating when the STM32 transmitter is idle.
uart_pin = digitalio.DigitalInOut(board.GP5)
uart_pin.direction = digitalio.Direction.INPUT
uart_pin.pull = digitalio.Pull.UP
time.sleep(0.1)
uart_pin.deinit() # Clean up pin state before handing over to the hardware peripheral

try:
    # In CircuitPython, for half-duplex listening on a single wire,
    # we pass None to TX, and GP1 to RX (matching the hardware connection).
    uart = busio.UART(None, board.GP1, baudrate=9600, timeout=10.0)
    print("UART Open-Drain Receiver initialized successfully!")
except Exception as e:
    print(f"UART Init Error: {e}")

# Variables for non-blocking LED toggling and data stream handling
led_last_toggle = time.monotonic()
led_state = False
input_buffer = bytearray()

while True:
    # Non-blocking heart-beat LED toggle every 0.5 seconds
    now = time.monotonic()
    if now - led_last_toggle >= 0.5:
        led_state = not led_state
        led.value = led_state
        led_last_toggle = now

    # UART stream reading
    if uart.in_waiting:
        data = uart.read(uart.in_waiting)
        if data:
            # Decode incoming bytes to UTF-8 string and print to the console
            print(data.decode("utf-8"), end="")
