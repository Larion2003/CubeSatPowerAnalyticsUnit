import board
import busio
import time

uart = busio.UART(board.GP0, board.GP1, baudrate=9600, timeout=1)

while True:
    uart.write("ping\n")

    # The shared single-wire bus reflects every transmitted byte back onto RX,
    # so the first readline() returns the echoed "ping" and the second one
    # returns the STM32's actual "pong\n" reply. Both must be read so the
    # receive buffer does not fill up.
    
    # Pico sends "ping\n":
    echo  = uart.readline()
    # STM sends "pong\n":
    reply = uart.readline()

    print("Raspberry Pi Pico W - (Master):", echo.decode().strip())
    print("STM32L431KC         - (Slave) :", reply.decode().strip())
      
