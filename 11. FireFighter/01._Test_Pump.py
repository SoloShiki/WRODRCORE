import time
import RPi.GPIO as GPIO
from hiwonder import RCCLite  # or RCControl depending on your library

# --- Setup ---
PUMP_PIN = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUMP_PIN, GPIO.OUT)

servo = RCCLite.Servo(channel=1)  # Adjust channel if needed

# --- Function to move servo like a firefighter ---
def spray_motion(duration=10, up_angle=60, down_angle=120, speed=0.5):
    start_time = time.time()
    GPIO.output(PUMP_PIN, GPIO.HIGH)  # Turn ON water pump
    print("Pump ON")

    try:
        while time.time() - start_time < duration:
            # Move servo up
            servo.set_angle(up_angle)
            time.sleep(speed)

            # Move servo down
            servo.set_angle(down_angle)
            time.sleep(speed)

    finally:
        # Stop everything
        GPIO.output(PUMP_PIN, GPIO.LOW)
        print("Pump OFF")
        servo.set_angle(90)  # Neutral position
        GPIO.cleanup()

# --- Run test ---
if __name__ == "__main__":
    print("Starting test...")
    spray_motion(duration=10)
    print("Test complete.")
