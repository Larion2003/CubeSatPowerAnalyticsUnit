import board
import busio
import time

uart = busio.UART(board.GP0,board.GP1,baudrate=9600)

while True:
    uart.write("ping\n")
    time.sleep(0.5)

# Startup code
#import board
#import busio
#import time
#import digitalio
#
#uart = busio.UART(board.GP0, board.GP1, baudrate=9600)
#
#while True:
#    uart.write(bytes("ping", "ascii"))
#    print("Pico: ping")
#    time.sleep(0.5)
