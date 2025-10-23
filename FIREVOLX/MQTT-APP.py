#!/usr/bin/env python3
# relay_mqtt_filter.py
# Código para recibir mensajes MQTT de detección de fuego/cigarrillos a CELULAR

import paho.mqtt.client as mqtt
import json

# --- Configuración ---
MQTT_BROKER = "192.168.149.148"      # IP del broker MQTT
MQTT_PORT = 1883
SOURCE_TOPIC = "alerta/fuego"      # Tópico de entrada (debe coincidir con el emisor)
DEST_TOPIC = "cigar/detect"        # Tópico de salida

# --- MODIFICADO ---
# Define las etiquetas que SÍ queremos reenviar
ALLOWED_LABELS = {"cigar", "fireball"}
# ------------------

# --- Lógica de conexión MQTT ---
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"🟢 Conectado al broker. Escuchando en: '{SOURCE_TOPIC}'")
        client.subscribe(SOURCE_TOPIC)
    else:
        print(f"❌ Falló la conexión. Código: {rc}")

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode()
        data = json.loads(payload)
        label = data.get("label")

        # --- LÓGICA DE FILTRADO MODIFICADA ---
        # Si la etiqueta no está en nuestro set de permitidos, la ignoramos.
        # Esto ignora "none" y cualquier otra cosa.
        if label not in ALLOWED_LABELS:
            # print(f"[relay] Ignorando mensaje con label: {label}") # Descomentar para depurar
            return  
        # -------------------------------------

        # --- MODIFICADO ---
        print(f"[relay] ✅ Detección válida ({label}). Reenviando mensaje: {payload}")
        # ------------------
        
        client.publish(DEST_TOPIC, json.dumps(data))
        print(f"[relay] Mensaje reenviado a '{DEST_TOPIC}'")

    except Exception as e:
        print(f"[relay] 🚨 Error procesando mensaje MQTT: {e}")

# --- Programa principal ---
if __name__ == "__main__":
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        print("[relay] 📡 Iniciando relay MQTT...")
        client.loop_forever()
    except KeyboardInterrupt:
        print("[relay] ⛔ Relay detenido por usuario.")
        client.disconnect()
    except Exception as e:
        print(f"[relay] 🚨 Error al conectar o ejecutar: {e}")