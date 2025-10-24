import time
import paho.mqtt.client as mqtt

BROKER_IP = "127.0.0.1" 
BROKER_PORT = 1883
TOPIC = "PUMP-TESTX" 

client = mqtt.Client()
client.connect(BROKER_IP, BROKER_PORT, 60)
client.loop_start()

state = True  # True = "on", False = "off"

try:
    while True:
        msg = "on" if state else "off"
        client.publish(TOPIC, msg)
        print(f"Published: {msg}")
        state = not state
        time.sleep(5)

except KeyboardInterrupt:
    print("Stopping program...")
    client.loop_stop()
    client.disconnect()    

