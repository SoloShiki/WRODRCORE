#!/usr/bin/env python3
# emisor_edgeimpulse_mqtt.py
# Raspberry Pi 4: publica detección de fuego + heartbeat "OK" cada 5 segundos.

import subprocess
import threading
import json
import time
import paho.mqtt.client as mqtt
import sys

# ---------- CONFIGURACIÓN ----------
BROKER_IP = "IP_PI5"       # IP de la Raspberry Pi 5
BROKER_PORT = 1883
MQTT_TOPIC = "alerta/fuego"
RPI_ID = "RPI_1"           # Etiqueta para identificar la zona o Pi
runner_path = '/usr/bin/edge-impulse-linux-runner'
DESIRED_LABEL = "fire"
THRESHOLD = 0.01
HEARTBEAT_INTERVAL = 5      # segundos
# -----------------------------------

# Configurar MQTT
client = mqtt.Client()
try:
    client.connect(BROKER_IP, BROKER_PORT, 60)
except Exception as e:
    print("[emisor] ERROR: No pude conectar al broker MQTT:", e)
    sys.exit(1)

client.loop_start()

# Variable compartida para indicar si se detectó fuego
fire_detected = False

def publish_fire_with_coords(box):
    global fire_detected
    fire_detected = True  # marcar que se detectó fuego
    x = box.get('x', 0)
    y = box.get('y', 0)
    width = box.get('width', 0)
    height = box.get('height', 0)
    center_x = x + width / 2
    center_y = y + height / 2

    payload = {
        "rpi_id": RPI_ID,
        "label": box.get('label', 'fire'),
        "confidence": box.get('value', 0.0),
        "x": x,
        "y": y,
        "width": width,
        "height": height,
        "center_x": center_x,
        "center_y": center_y
    }

    try:
        client.publish(MQTT_TOPIC, json.dumps(payload))
        print(f"[emisor] Mensaje FIRE enviado: {payload}")
    except Exception as e:
        print("[emisor] Error publicando MQTT:", e)

def publish_heartbeat():
    """Envía un mensaje 'OK' cada HEARTBEAT_INTERVAL segundos si no hay fuego."""
    global fire_detected
    while True:
        if not fire_detected:
            payload = {
                "rpi_id": RPI_ID,
                "label": "none",
                "status": "OK"
            }
            try:
                client.publish(MQTT_TOPIC, json.dumps(payload))
                print(f"[emisor] Mensaje HEARTBEAT enviado: {payload}")
            except Exception as e:
                print("[emisor] Error publicando heartbeat:", e)
        else:
            fire_detected = False  # reiniciar detección para la próxima iteración
        time.sleep(HEARTBEAT_INTERVAL)

# Lanza el runner de Edge Impulse
try:
    proc = subprocess.Popen(
        [runner_path, '--clean', '--camera', '/dev/video0'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )
    print("[emisor] Runner lanzado.")
except Exception as e:
    print("[emisor] No se pudo lanzar edge-impulse-linux-runner:", e)
    sys.exit(1)

def read_loop():
    for line in proc.stdout:
        line = line.strip()
        if not line:
            continue
        print("[runner]", line)
        if 'boundingBoxes' in line:
            try:
                json_start = line.find('[')
                json_data = line[json_start:]
                boxes = json.loads(json_data)
                for box in boxes:
                    label = box.get('label', '')
                    value = box.get('value', 0.0)
                    if label == DESIRED_LABEL and value >= THRESHOLD:
                        print(f"[emisor] Detectado {label} con confianza {value:.2f}")
                        publish_fire_with_coords(box)
                        time.sleep(5)  # evita múltiples publicaciones continuas
            except Exception as e:
                print("[emisor] Error analizando boundingBoxes:", e)

# Iniciar hilos
reader = threading.Thread(target=read_loop, daemon=True)
reader.start()

heartbeat_thread = threading.Thread(target=publish_heartbeat, daemon=True)
heartbeat_thread.start()

# Monitor básico
try:
    while True:
        time.sleep(1)
        if proc.poll() is not None:
            print("[emisor] El runner se cerró.")
            break
except KeyboardInterrupt:
    print("[emisor] Interrumpido por usuario.")
finally:
    try:
        proc.kill()
    except:
        pass
    client.loop_stop()
    client.disconnect()
