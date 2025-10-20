#!/usr/bin/env python3
from gpiozero import Servo, DigitalOutputDevice
from time import sleep, time

# --- Pins ---
SERVO_PIN = 24
PUMP_PIN = 23

# --- Motion parameters ---
CYCLE_SPEED = 0.5        # seconds per motion step
MOTION_DURATION = 10.0   # total duration
ANGLE_150 = 0.5          # gpiozero servo value for ~150°
ANGLE_170 = 0.7          # gpiozero servo value for ~170°
ANGLE_NEUTRAL = 0.5      # neutral/center

def main():
    # --- Initialize hardware ---
    servo = Servo(SERVO_PIN)
    pump = DigitalOutputDevice(PUMP_PIN)

    # Turn pump ON
    pump.on()
    print("💧 Pump ON. Spray motion started.")

    start_time = time()
    is_up_position = False  # start with 150°

    try:
        while (time() - start_time) < MOTION_DURATION:
            # Alternate servo positions
            if is_up_position:
                servo.value = ANGLE_170
                is_up_position = False
            else:
                servo.value = ANGLE_150
                is_up_position = True

            print(f"→ Servo moved to {servo.value}")
            sleep(CYCLE_SPEED)

        # Motion complete: center servo and turn off pump
        print("✅ Motion complete. Pump OFF. Centering servo...")
        servo.value = ANGLE_NEUTRAL
        sleep(1)
        pump.off()

    except KeyboardInterrupt:
        print("⚠️ Interrupted by user. Pump OFF.")
        pump.off()

    finally:
        print("🧹 Done.")

if __name__ == "__main__":
    main()
