import time
import paho.mqtt.client as mqtt

# MQTT setup
client = mqtt.Client()
client.connect("192.168.149.1", 1883, 60)  # Replace with your host IP
client.loop_start()  # Start background network loop

state = True  # True = "on", False = "off"

try:
    while True:
        # Toggle state
        msg = "on" if state else "off"
        client.publish("motor_pump_cmd", msg)
        print(f"Published: {msg}")
        state = not state  # Flip state for next iteration

        # Wait 5 seconds
        time.sleep(5)

except KeyboardInterrupt:
    print("Stopping toggle program...")
    client.loop_stop()
    client.disconnect()