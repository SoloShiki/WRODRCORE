#!/usr/bin/env python3
import time
import paho.mqtt.client as mqtt
from gpiozero import Servo, DigitalOutputDevice
from time import sleep

# -----------------------------
# MQTT CONFIG
# -----------------------------
BROKER_IP = "127.0.0.1"  
BROKER_PORT = 1883
TOPIC = "PUMP-TESTX"

# -----------------------------
# GPIO CONFIG
# -----------------------------
SERVO_PIN = 24   # Servo pin
PUMP_PIN = 23    # Pump pin

CYCLE_SPEED = 0.5        # seconds per step
MOTION_DURATION = 10.0   # total motion duration (seconds)
ANGLE_150 = 0.5          # approx 150Â°
ANGLE_170 = 0.7          # approx 170Â°
ANGLE_NEUTRAL = 0.5      # neutral position

# -----------------------------
# Initialize hardware
# -----------------------------
servo = Servo(SERVO_PIN)
pump = DigitalOutputDevice(PUMP_PIN)

# -----------------------------
# MQTT CALLBACKS
# -----------------------------
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe(TOPIC)

def run_pump_cycle():
    """Run the pump + servo movement cycle."""
    print("âš¡ Activating pump and servo cycle...")
    pump.on()
    start_time = time.time()
    is_up_position = False

    try:
        while (time.time() - start_time) < MOTION_DURATION:
            if is_up_position:
                servo.value = ANGLE_170
                is_up_position = False
            else:
                servo.value = ANGLE_150
                is_up_position = True
            sleep(CYCLE_SPEED)
    except KeyboardInterrupt:
        print("âš  Manual interrupt detected. Stopping pump.")
    finally:
        servo.value = ANGLE_NEUTRAL
        pump.off()
        print("âœ… Pump cycle completed.")

def on_message(client, userdata, msg):
    command = msg.payload.decode()
    print(f"Received message: {command}")
    if command.lower() == "on":
        run_pump_cycle()
    elif command.lower() == "off":
        print("Stopping pump manually...")
        pump.off()
        servo.value = ANGLE_NEUTRAL

# -----------------------------
# MAIN MQTT CLIENT
# -----------------------------
client = mqtt.Client(protocol=mqtt.MQTTv311)
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER_IP, BROKER_PORT, 60)
client.loop_forever()
