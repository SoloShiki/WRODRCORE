#!/usr/bin/env python3
import time
from your_sdk_file import Board  # replace with the filename you saved the SDK as, e.g., import sdk

def main():
    board = Board(device="/dev/ttyUSB1")  # use your USB device
    board.enable_reception(True)           # start listening to the board

    # Blink LED 3 times
    board.set_led(on_time=0.2, off_time=0.2, repeat=3, led_id=1)

    # Buzzer beep: 1500Hz, 0.3s on, 0.1s off, repeat 2 times
    board.set_buzzer(freq=1500, on_time=0.3, off_time=0.1, repeat=2)

    print("Commands sent, check your board!")

    # Keep the script alive to receive board feedback (optional)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")

if __name__ == "__main__":
    main()