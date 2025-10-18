import time
#import RPi.GPIO as GPIO
#from hiwonder import RCCLite

# -----------------------------
# Configuration
# -----------------------------
PUMP_PIN = 23
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(PUMP_PIN, GPIO.OUT)

#servo = RCCLite.Servo(channel=1)

# -----------------------------
# State Machine Class
# -----------------------------
class FireFighterRobot:
    def __init__(self):
        self.state = "IDLE"
        self.alarm_triggered = False
        self.fire_detected = False
        self.fire_location = None
        print("Robot initialized. Current state: IDLE")

    def run(self):
        while True:
            if self.state == "IDLE":
                self.idle_state()
            elif self.state == "NAVIGATION":
                self.navigation_state()
            elif self.state == "SEARCH":
                self.search_state()
            elif self.state == "FIRE_FIGHTING":
                self.fire_fighting_state()
            elif self.state == "DONE":
                self.done_state()
                break
            else:
                print("Unknown state! Stopping.")
                break
            time.sleep(0.1)

    # -----------------------------
    # State Definitions
    # -----------------------------


def idle_state(self):
    print("[STATE] IDLE: Running mosquito control program...")

    # Start rpi_mosquito.py if not already running
    if not hasattr(self, 'mosquito_process') or self.mosquito_process is None:
        try:
            self.mosquito_process = subprocess.Popen(
                ["python3", "programs/rpi_mosquito.py"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print("rpi_mosquito.py started.")
        except Exception as e:
            print(f"Failed to start rpi_mosquito.py: {e}")
            self.mosquito_process = None


    # Simulate alarm trigger
    if self.check_alarm():
        print("Alarm detected — stopping mosquito program.")
        #DECTTAR ALARMA AQUI
        self.stop_mosquito_program()
        self.state = "NAVIGATION"
        
            





    def navigation_state(self):        
        print("[STATE] NAVIGATION: Running Mecanum_Maze_A.py for navigation...")
 
    try:
        # Run the external navigation program
        result = subprocess.run(
            ["python3", "programs/Mecanum_Maze_A.py"],
            check=True,
            capture_output=True,
            text=True
        )
        print("Navigation program output:")
        print(result.stdout)

    except subprocess.CalledProcessError as e:
        print("Error running Mecanum_Maze_A.py:")
        print(e.stderr)
        self.state = "IDLE"
        return

    print("Navigation completed successfully.")
    self.state = "SEARCH"

    def search_state(self):
        print("[STATE] SEARCH: Scanning area for fire...")
        # Simulate vision detection (replace with camera + flame detection)
        time.sleep(2)
        if self.detect_fire():
            print("Fire detected!")
            self.state = "FIRE_FIGHTING"
        else:
            print("No fire found, returning to IDLE.")
            self.state = "IDLE"

    def fire_fighting_state(self):
        print("[STATE] FIRE_FIGHTING: Engaging water pump and servo...")
        self.activate_pump(True)
        start_time = time.time()

        try:
            while time.time() - start_time < 10:
                #servo.set_angle(60)
                time.sleep(0.5)
                #servo.set_angle(120)
                time.sleep(0.5)
            print("Fire extinguished (assumed).")
        finally:
            self.activate_pump(False)
            #servo.set_angle(90)
            self.state = "DONE"

    def done_state(self):
        print("[STATE] DONE: Task complete. Returning to base or idle.")
        self.cleanup()

    # -----------------------------
    # Helper Methods
    # -----------------------------

    def check_alarm(self):
        # TODO: Replace with actual alarm input (GPIO, MQTT, etc.)
        # Simulated trigger for demo
        time.sleep(2)
        print("Alarm triggered!")
        return True

    def detect_fire(self):
        # TODO: Replace with vision logic
        self.fire_detected = True
        return True

    def activate_pump(self, on):
        #GPIO.output(PUMP_PIN, GPIO.HIGH if on else GPIO.LOW)
        print("Pump", "ON" if on else "OFF")

    def cleanup(self):
        #GPIO.output(PUMP_PIN, GPIO.LOW)
        #GPIO.cleanup()
        print("System shutdown complete.")

# -----------------------------
# Main
# -----------------------------
if __name__ == "__main__":
    robot = FireFighterRobot()
    robot.run()
