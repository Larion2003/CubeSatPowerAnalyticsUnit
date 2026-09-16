import board
import busio
import time

uart = busio.UART(board.GP0, board.GP1, baudrate=9600)

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

    try:
        if echo:
            print("Raspberry Pi Pico W - (Master):", echo.decode().strip())
        if reply:
            print("STM32L431KC         - (Slave) :", reply.decode().strip())
    except UnicodeError:
        # A shared single-wire bus occasionally picks up noise, corrupting
        # one byte into an invalid UTF-8 sequence. 
        print("Warning: corrupted byte received on the UART line, skipping")
    
    # For debug    
    #time.sleep(0.1)
