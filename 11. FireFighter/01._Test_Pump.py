#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# --- Pin setup ---
SERVO_PIN = 24     # Servo signal pin
PUMP_PIN = 23      # Pump relay control

# --- Motion settings ---
CYCLE_SPEED = 0.5       # seconds between servo moves
MOTION_DURATION = 10.0   # total duration (seconds)
ANGLE_150 = 150
ANGLE_170 = 170
ANGLE_NEUTRAL = 150      # neutral position

# --- Helper function: angle to duty cycle ---
def angle_to_duty_cycle(angle):
    """Convert 0–180° to PWM duty cycle (for 50 Hz)"""
    return 2.5 + (angle / 180.0) * 10.0

def main():
    # --- Setup GPIO ---
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    GPIO.setup(PUMP_PIN, GPIO.OUT)

    # Servo: 50 Hz PWM
    servo_pwm = GPIO.PWM(SERVO_PIN, 50)
    servo_pwm.start(angle_to_duty_cycle(ANGLE_NEUTRAL))

    # Turn pump ON
    GPIO.output(PUMP_PIN, GPIO.HIGH)
    print("💧 Spray motion started. Pump ON.")

    start_time = time.time()
    is_up = False

    try:
        while (time.time() - start_time) < MOTION_DURATION:
            # Alternate angles
            if is_up:
                target = ANGLE_170
                is_up = False
            else:
                target = ANGLE_150
                is_up = True

            duty = angle_to_duty_cycle(target)
            servo_pwm.ChangeDutyCycle(duty)
            print(f"→ Servo to {target}° (duty {duty:.2f}%)")

            time.sleep(CYCLE_SPEED)

        # After motion complete
        print("✅ Motion complete. Pump OFF. Centering servo...")
        servo_pwm.ChangeDutyCycle(angle_to_duty_cycle(ANGLE_NEUTRAL))
        time.sleep(1)
        GPIO.output(PUMP_PIN, GPIO.LOW)

    except KeyboardInterrupt:
        print("⚠️ Interrupted by user. Cleaning up...")
        GPIO.output(PUMP_PIN, GPIO.LOW)

    finally:
        servo_pwm.stop()
        GPIO.cleanup()
        print("🧹 GPIO cleaned up. Exiting.")

if __name__ == "__main__":
    main()
