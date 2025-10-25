#/usr/bin/env python3
import time
import subprocess
import json
import threading
import paho.mqtt.client as mqtt

# -X----------------------------
# CONFIGURACI ^sN MQTT
# -----------------------------
BROKERS = ["192.168.149.171", "192.168.149.148"]
BROKER_PORT = 1883
TOPIC_FIRE = "alerta/fuego"

# -----------------------------
#x Rutas de scripts
# -----------------------------
NAVIGATION_SCRIPT = "/home/ubuntu/firevolx_ws/src/firevolx/firevolx/NAVEGATION" # Script de Navegacion 
FIRE_FIGHT_SCRIPT = "/home/ubuntu/firevolx_ws/src/firevolx/firevolx/TEST-MQTT"  # Script de bomba MQTT
SERVO_SCRIPT = "/home/ubuntu/firevolx_ws/src/firevolx/firevolx/SERVO-TEST"       # Nodo ROS2 del servo

# -----------------------------
# FireFighterRobot: M  quina de estado
# -----------------------------
class FireFighterRobot:
    def __init__(self):
        self.state = "IDLE"
        self.fire_detected = False
        self.last_rpi = None
        print(" ^= ^v Robot iniciado. Estado actual: IDLE")

        # Conectar a todos los brokers MQTT
        self.mqtt_clients = []
        for ip in BROKERS:
            client = mqtt.Client()
            client.on_connect = self.on_connect
            client.on_message = self.on_message
            try:
                client.connect(ip, BROKER_PORT, 60)
                print(f"[MQTT] Conectado al broker {ip}:{BROKER_PORT}")
            except Exception as e:
                print(f"[MQTT] Error al conectar al broker {ip}: {e}")
            client.loop_start()
            self.mqtt_clients.append(client)

    # -----------------------------
    # MQTT Callbacksss
    # -----------------------------
    def on_connect(self, client, userdata, flags, rc):
        print(f"[MQTT] Conectado. Subscrito al t  pico: {TOPIC_FIRE}")
        client.subscribe(TOPIC_FIRE)

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            label = data.get("label")
            rpi_id = data.get("rpi_id")

            if label in ["fire", "cigar", "fireball"]:
                print(f"[MQTT] ^=^z Fuego detectado por {rpi_id}: {label}")
                self.fire_detected = True
                self.last_rpi = rpi_id
            else:
                print(f"[MQTT] Mensaje ignorado de {rpi_id}: {label}")
        except Exception as e:
            print("[MQTT] Error procesando mensaje:", e)

    # -----------------------------
    # L  gica de estados
    # -----------------------------
    def run(self):
        while True:
            if self.state == "IDLE" and self.fire_detected:
                print("[STATE] Se  al de fuego recibida. Cambiando a NAVIGATION.")
                self.state = "NAVIGATION"

            elif self.state == "NAVIGATION":
                self.navigation_state()
                self.state = "SEARCH"

            elif self.state == "SEARCH":
                self.search_state()
                self.state = "FIRE_FIGHTING"

            elif self.state == "FIRE_FIGHTING":
                self.fire_fighting_state()
                self.state = "DONE"

            elif self.state == "DONE":
                self.done_state()
                self.state = "IDLE"

            time.sleep(0.2)

    # -----------------------------
    # ESTADOS INDIVIDUALES
    # -----------------------------
    def navigation_state(self):
        print(f"[STATE] NAVIGATION: Ejecutando navegaci  n por {self.last_rpi}...")
        try:
            #subprocess.run(["python3", NAVIGATION_SCRIPT], check=True)   #changed below
            subprocess.run(["python3", NAVIGATION_SCRIPT, str(self.last_rpi)], check=True)
        except Exception as e:
            print(f"Error ejecutando navegaci  n: {e}")

    def search_state(self):
        print("[STATE] SEARCH: Escaneando   rea...")
        time.sleep(2)
        print("Fuego confirmado visualmente.")

    def fire_fighting_state(self):
        print("[STATE] FIRE_FIGHTING: Activando bomba de agua y servos simult  neamente...")

        try:
            # Lanzar la bomba y el servo en paralelo
            pump_process = subprocess.Popen(["python3", FIRE_FIGHT_SCRIPT])
            servo_process = subprocess.Popen(["python3", SERVO_SCRIPT])

            print("[ACTION] Bomba y servo iniciados al mismo tiempo.")

            # Esperar un tiempo mientras trabajan (puedes ajustar este valor)
            time.sleep(5)

            # Finalizar ambos procesos
            pump_process.terminate()
            servo_process.terminate()

            print("[ACTION] Operaci  n de extinci  n completada.")

        except Exception as e:
            print(f"[ERROR] Error ejecutando scripts simult  neos: {e}")

    def done_state(self):
        print("[STATE] DONE: Operaci  n completada. Regresando a IDLE.")
        self.fire_detected = False
        self.last_rpi = None


# -----------------------------
# Main
# -----------------------------
if __name__ == "__main__":
    robot = FireFighterRobot()
    robot.run()











