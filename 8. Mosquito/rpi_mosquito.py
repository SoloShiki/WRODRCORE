#!/usr/bin/env python3
# receptor_mqtt_fire_action.py
# Ejecutar en Raspberry Pi 5. Recibe detección de fuego vía MQTT y ejecuta un programa externo.
# Ahora también imprime un mensaje cuando recibe heartbeat "OK" para testing.

import paho.mqtt.client as mqtt
import json
import subprocess

# ---------- CONFIGURACIÓN ----------
BROKER_IP = "IP_PI5"       # IP de esta Raspberry Pi 5
BROKER_PORT = 1883
MQTT_TOPIC = "alerta/fuego"
EXTERNAL_PROGRAM = "/home/pi/fire_action.py"  # Python program to run when fire is detected
# -----------------------------------

def on_connect(client, userdata, flags, rc):
    print("[receptor] Conectado al broker MQTT con código:", rc)
    client.subscribe(MQTT_TOPIC)

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode()
        data = json.loads(payload)
        label = data.get("label", "none")
        
        if label == "none":
            # Heartbeat message
            print(f"[receptor] HEARTBEAT recibido de {data.get('rpi_id', 'unknown')} → estado OK")
            return

        if label == "fire":
            rpi_id = data.get("rpi_id", "unknown")
            x = data.get("x", 0)
            y = data.get("y", 0)
            print(f"[receptor] Fuego detectado desde {rpi_id} en coordenadas x={x}, y={y}")

            # Ejecutar el programa externo con argumentos
            subprocess.Popen(["python3", EXTERNAL_PROGRAM, rpi_id, str(x), str(y)])

    except Exception as e:
        print("[receptor] Error procesando mensaje MQTT:", e)

def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER_IP, BROKER_PORT, 60)
    client.loop_forever()

if __name__ == '__main__':
    main()
