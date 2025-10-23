import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO

GPIO_PIN = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_PIN, GPIO.OUT, initial=GPIO.LOW)

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    if payload.lower() == "on":
        GPIO.output(GPIO_PIN, True)
        print("Motor pump ON")
    elif payload.lower() == "off":
        GPIO.output(GPIO_PIN, False)
        print("Motor pump OFF")

client = mqtt.Client()
client.on_message = on_message
client.connect("localhost", 1883, 60)
client.subscribe("motor_pump_cmd")
client.loop_forever()