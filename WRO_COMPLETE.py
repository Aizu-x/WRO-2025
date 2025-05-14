import RPi.GPIO as GPIO
import time
import json
import subprocess
from gpiozero import Motor, PWMOutputDevice
import sys

# Start the camera object detection process
camera_process = subprocess.Popen([
    "python3", "imx500_object_detection_demo.py",
    "--model", "/home/roshdi-24/picamera2/network.rpk",
    "--labels", "/home/roshdi-24/picamera2/labels.txt",
    "--fps", "25",
    "--bbox-normalization",
    "--ignore-dash-labels",
    "--bbox-order", "xy"
], cwd="/home/roshdi-24/picamera2/examples/imx500/")

# Allow camera time to start up
time.sleep(5)

# Vehicle control class
class Vehicle:
    def __init__(self):
        self.left_motor = Motor(forward=17, backward=27)
        self.left_pwm = PWMOutputDevice(14)
        self.left_pwm.value = 0

        self.right_motor = Motor(forward=22, backward=23)
        self.right_pwm = PWMOutputDevice(15)
        self.right_pwm.value = 0

    def forward(self, speed=0.85):
        self.left_motor.backward()
        self.left_pwm.value = speed
        self.right_motor.backward()
        self.right_pwm.value = speed

    def backward(self, speed=0.85):
        self.left_motor.forward()
        self.left_pwm.value = speed
        self.right_motor.forward()
        self.right_pwm.value = speed

    def leftturn(self):
        self.right_motor.backward()
        self.right_pwm.value = 1
        self.left_motor.forward()
        self.left_pwm.value = 1

    def rightturn(self):
        self.left_motor.backward()
        self.left_pwm.value = 1
        self.right_motor.forward()
        self.right_pwm.value = 1

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
        self.left_pwm.value = 0
        self.right_pwm.value = 0

# GPIO setup for ultrasonic sensors
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

TRIG_LEFT_FRONT = 24
ECHO_LEFT_FRONT = 25
TRIG_RIGHT_FRONT = 2
ECHO_RIGHT_FRONT = 3

GPIO.setup(TRIG_LEFT_FRONT, GPIO.OUT)
GPIO.setup(ECHO_LEFT_FRONT, GPIO.IN)
GPIO.setup(TRIG_RIGHT_FRONT, GPIO.OUT)
GPIO.setup(ECHO_RIGHT_FRONT, GPIO.IN)

def get_distance(trig_pin, echo_pin):
    GPIO.output(trig_pin, GPIO.LOW)
    time.sleep(0.0002)
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    pulse_start = time.time()
    timeout_start = time.time()
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
        if pulse_start - timeout_start > 0.05:
            return 999

    pulse_end = time.time()
    timeout_start = time.time()
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()
        if pulse_end - timeout_start > 0.05:
            return 999

    duration = pulse_end - pulse_start
    distance = duration * 17150
    return round(distance, 2)

# Parking function
def park_and_shutdown():
    print("Parking sequence started...")
    rpi_vehicle.forward(speed=0.4)

    while True:
        dist_left = get_distance(TRIG_LEFT_FRONT, ECHO_LEFT_FRONT)
        dist_right = get_distance(TRIG_RIGHT_FRONT, ECHO_RIGHT_FRONT)
        min_dist = min(dist_left, dist_right)

        print(f"Parking... distance: {min_dist:.1f} cm")

        if min_dist < 8:
            print("Reached parking spot. Stopping and shutting down.")
            rpi_vehicle.stop()
            GPIO.cleanup()
            camera_process.terminate()
            sys.exit(0)

        time.sleep(0.2)

# Create vehicle instance
rpi_vehicle = Vehicle()

try:
    while True:
        label = ""

        try:
            with open("/home/roshdi-24/picamera2/results/detections.json", "r") as f:
                data = json.load(f)
                if isinstance(data, list) and len(data) > 0:
                    label = data[0].get("label", "").lower()
        except Exception as e:
            print("Error reading JSON:", e)

        dist_left_front = get_distance(TRIG_LEFT_FRONT, ECHO_LEFT_FRONT)
        dist_right_front = get_distance(TRIG_RIGHT_FRONT, ECHO_RIGHT_FRONT)
        obstacle_threshold = 10.0

        print(f"Label: {label} | Left Front: {dist_left_front:.1f} cm | Right Front: {dist_right_front:.1f} cm")

        if dist_left_front < obstacle_threshold or dist_right_front < obstacle_threshold:
            print("Obstacle detected! Reversing and turning.")
            rpi_vehicle.backward()
            time.sleep(0.5)
            rpi_vehicle.stop()

            if dist_left_front > dist_right_front:
                rpi_vehicle.leftturn()
            else:
                rpi_vehicle.rightturn()
            time.sleep(1.3)
            rpi_vehicle.stop()

        elif "redbox" in label:
            print("Red box detected → Turning right.")
            rpi_vehicle.rightturn()
            time.sleep(1.2)
            rpi_vehicle.forward()
            time.sleep(1.0)
            rpi_vehicle.leftturn()
            time.sleep(0.7)
            rpi_vehicle.stop()

        elif "greenbox" in label:
            print("Green box detected → Turning left.")
            rpi_vehicle.leftturn()
            time.sleep(1.2)
            rpi_vehicle.forward()
            time.sleep(1.0)
            rpi_vehicle.rightturn()
            time.sleep(0.7)
            rpi_vehicle.stop()

        elif "xparking" in label:
            print("Parking area detected → Initiating parking.")
            park_and_shutdown()

        else:
            print("Moving forward.")
            rpi_vehicle.forward()

        time.sleep(0.2)

except KeyboardInterrupt:
    print("Manual shutdown triggered.")
    rpi_vehicle.stop()
    GPIO.cleanup()
    camera_process.terminate()
