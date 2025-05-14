import RPi.GPIO as GPIO
import time
import json
import subprocess
from gpiozero import Motor, PWMOutputDevice
import sys

# Start object detection camera process
camera_process = subprocess.Popen([
    "python3", "imx500_object_detection_demo.py",
    "--model", "/home/roshdi-24/picamera2/network.rpk",
    "--labels", "/home/roshdi-24/picamera2/labels.txt",
    "--fps", "25",
    "--bbox-normalization",
    "--ignore-dash-labels",
    "--bbox-order", "xy"
], cwd="/home/roshdi-24/picamera2/examples/imx500/")

time.sleep(5)

# === Motor Setup ===
class Vehicle:
    def __init__(self):
        self.left_motor = Motor(forward=17, backward=27)
        self.left_pwm = PWMOutputDevice(14)
        self.right_motor = Motor(forward=22, backward=23)
        self.right_pwm = PWMOutputDevice(15)
        self.set_speed(0)

    def set_speed(self, speed):
        self.left_pwm.value = speed
        self.right_pwm.value = speed

    def forward(self, speed=0.8):
        self.set_speed(speed)
        self.left_motor.backward()
        self.right_motor.backward()

    def backward(self, speed=0.8):
        self.set_speed(speed)
        self.left_motor.forward()
        self.right_motor.forward()

    def leftturn(self, speed=0.7):
        self.left_motor.forward()
        self.right_motor.backward()
        self.left_pwm.value = speed
        self.right_pwm.value = 1

    def rightturn(self, speed=0.7):
        self.left_motor.backward()
        self.right_motor.forward()
        self.left_pwm.value = 1
        self.right_pwm.value = speed

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
        self.set_speed(0)

# === Ultrasonic Sensor Setup ===
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

TRIG_LEFT = 24
ECHO_LEFT = 25
TRIG_RIGHT = 2
ECHO_RIGHT = 3

GPIO.setup(TRIG_LEFT, GPIO.OUT)
GPIO.setup(ECHO_LEFT, GPIO.IN)
GPIO.setup(TRIG_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_RIGHT, GPIO.IN)

def get_distance(trig, echo):
    GPIO.output(trig, GPIO.LOW)
    time.sleep(0.0002)
    GPIO.output(trig, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig, GPIO.LOW)

    timeout_start = time.time()
    while GPIO.input(echo) == 0:
        if time.time() - timeout_start > 0.05:
            return 999

    pulse_start = time.time()
    while GPIO.input(echo) == 1:
        if time.time() - timeout_start > 0.05:
            return 999
    pulse_end = time.time()

    duration = pulse_end - pulse_start
    return round(duration * 17150, 2)

# === Parking ===
def park_and_shutdown():
    print("\U0001F17F️ Parking zone detected. Starting parking sequence...")
    rpi.forward(0.4)
    while True:
        left = get_distance(TRIG_LEFT, ECHO_LEFT)
        right = get_distance(TRIG_RIGHT, ECHO_RIGHT)
        if min(left, right) < 8:
            rpi.stop()
            print("✅ Parked! Shutting down.")
            GPIO.cleanup()
            camera_process.terminate()
            sys.exit(0)
        time.sleep(0.2)

# === Lap Tracking ===
lap_count = 0
last_lap_time = time.time()
LAP_GAP = 7
LAP_DISTANCE_THRESHOLD = 5

rpi = Vehicle()
obstacle_threshold = 10.0

try:
    while True:
        label = ""

        try:
            with open("/home/roshdi-24/picamera2/results/detections.json", "r") as f:
                data = json.load(f)
                if isinstance(data, list) and data:
                    label = data[0].get("label", "").lower()
        except Exception as e:
            print("⚠️ JSON read error:", e)

        dist_left = get_distance(TRIG_LEFT, ECHO_LEFT)
        dist_right = get_distance(TRIG_RIGHT, ECHO_RIGHT)

        print(f"🧠 Label: {label} | L: {dist_left:.1f} cm | R: {dist_right:.1f} cm")

        # === Lap Detection ===
        if dist_left < LAP_DISTANCE_THRESHOLD and dist_right < LAP_DISTANCE_THRESHOLD:
            now = time.time()
            if now - last_lap_time > LAP_GAP:
                lap_count += 1
                last_lap_time = now
                print(f"🏁 Lap {lap_count} completed")
            if lap_count >= 3:
                print("🎉 All 3 laps completed! Stopping.")
                rpi.stop()
                GPIO.cleanup()
                camera_process.terminate()
                sys.exit(0)

        # === Parking Detection ===
        elif "xparking" in label:
            park_and_shutdown()

        # === Obstacle Avoidance ===
        elif dist_left < obstacle_threshold or dist_right < obstacle_threshold:
            print("⛔ Obstacle! Avoiding...")
            rpi.backward(0.6)
            time.sleep(0.3)
            rpi.stop()
            if dist_left > dist_right:
                rpi.leftturn()
            else:
                rpi.rightturn()
            time.sleep(1.0)
            rpi.stop()

        # === Box Detection ===
        elif "redbox" in label:
            print("🔴 Red Box → Turning right")
            rpi.rightturn()
            time.sleep(1.2)
            rpi.forward()
            time.sleep(1.0)
            rpi.leftturn()
            time.sleep(0.7)
            rpi.stop()

        elif "greenbox" in label:
            print("🟢 Green Box → Turning left")
            rpi.leftturn()
            time.sleep(1.2)
            rpi.forward()
            time.sleep(1.0)
            rpi.rightturn()
            time.sleep(0.7)
            rpi.stop()

        # === Default Movement ===
        else:
            print("🚗 Moving forward")
            rpi.forward()

        time.sleep(0.2)

except KeyboardInterrupt:
    print("🛑 Manual stop")
    rpi.stop()
    GPIO.cleanup()
    camera_process.terminate()
