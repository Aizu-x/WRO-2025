import RPi.GPIO as GPIO
import time
import smbus2
import json
import subprocess
import sys
from gpiozero import Motor, PWMOutputDevice

# === Start IMX500 Object Detection Process (Camera) ===
camera_process = subprocess.Popen([
    "python3", "imx500_object_detection_demo.py",
    "--model", "/home/roshdi-24/picamera2/network.rpk",
    "--labels", "/home/roshdi-24/picamera2/labels.txt",
    "--fps", "25",
    "--bbox-normalization",
    "--ignore-dash-labels",
    "--bbox-order", "xy"
], cwd="/home/roshdi-24/picamera2/examples/imx500/")
time.sleep(5)  # Allow time for the camera to initialize

# === GPIO Setup ===
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# === Servo Setup :) ===
SERVO_PIN = 4
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 is servo frequency
pwm.start(0)

# Steering angles (tuned for clean left/right turns)
STEER_LEFT_ANGLE = 45
STEER_RIGHT_ANGLE = 135
STEER_CENTER_ANGLE = 90
current_angle = STEER_CENTER_ANGLE

steer_count = 0  # Track total number of left + right turns

def set_servo_angle(angle): #my func. for steering angle adjustment
    global current_angle
    duty_cycle = (angle / 18) + 2.5
    pwm.ChangeDutyCycle(duty_cycle)
    current_angle = angle
    print(f"🔄 Steering to angle: {angle}") #always used print for debugging and testing response
    time.sleep(0.3)

def steer_left():
    global steer_count
    steer_count += 1
    print(f"↩️ Steer Left (Total Turns: {steer_count})")
    if steer_count >= 12:
        print("🛑 Reached 12 steering actions. Shutting down.")
        rpi.stop()
        steer_center()
        pwm.stop()
        GPIO.cleanup()
        camera_process.terminate()
        sys.exit(0)
    set_servo_angle(STEER_LEFT_ANGLE)

def steer_right():
    global steer_count
    steer_count += 1
    print(f"↪️ Steer Right (Total Turns: {steer_count})")
    if steer_count >= 12:
        print("🛑 Reached 12 steering actions. Shutting down.")
        rpi.stop()
        steer_center()
        pwm.stop()
        GPIO.cleanup()
        camera_process.terminate()
        sys.exit(0)
    set_servo_angle(STEER_RIGHT_ANGLE)

def steer_center():
    set_servo_angle(STEER_CENTER_ANGLE)

# === Front Ultrasonic Sensor, For obstacle detection straight ahead
TRIG = 23
ECHO = 24
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_front_distance(): # will get the front distance and it includes delay so it lessens the garbage values
    GPIO.output(TRIG, False)
    time.sleep(0.0002)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    timeout_start = time.time()
    while GPIO.input(ECHO) == 0:
        if time.time() - timeout_start > 0.05:
            return 999
    pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        if time.time() - timeout_start > 0.05:
            return 999
    pulse_end = time.time()

    return round((pulse_end - pulse_start) * 17150, 2)

# === Right Side Ultrasonic Sensor added for smart wall-avoidance a working technique 
SIDE_TRIG = 5
SIDE_ECHO = 6
GPIO.setup(SIDE_TRIG, GPIO.OUT)
GPIO.setup(SIDE_ECHO, GPIO.IN)

def get_side_distance(): # similar to the front distance func.
    GPIO.output(SIDE_TRIG, False)
    time.sleep(0.0002)
    GPIO.output(SIDE_TRIG, True)
    time.sleep(0.00001)
    GPIO.output(SIDE_TRIG, False)

    timeout_start = time.time()
    while GPIO.input(SIDE_ECHO) == 0:
        if time.time() - timeout_start > 0.05:
            return 999
    pulse_start = time.time()

    while GPIO.input(SIDE_ECHO) == 1:
        if time.time() - timeout_start > 0.05:
            return 999
    pulse_end = time.time()

    return round((pulse_end - pulse_start) * 17150, 2)

# === ISL29125 Color Sensor Setup detects blue or orange — used for steering cues specially for knowing if we are going counterclockwise or clockwise 
ISL29125_ADDRESS = 0x44
CONFIG_1 = 0x01
CONFIG_2 = 0x02
CONFIG_3 = 0x03
RED_L = 0x09
GREEN_L = 0x0B
BLUE_L = 0x0D
bus = smbus2.SMBus(1)

def write_register(register, value): # turns on and sets the mode for color sensor 
    bus.write_byte_data(ISL29125_ADDRESS, register, value)

def read_word(register): # seperates readings to each different sections (RED GREEN BLUE)
    low = bus.read_byte_data(ISL29125_ADDRESS, register)
    high = bus.read_byte_data(ISL29125_ADDRESS, register + 1)
    return (high << 8) | low

def init_isl29125():
    write_register(CONFIG_1, 0x05)
    write_register(CONFIG_2, 0x00)
    write_register(CONFIG_3, 0x00)
    time.sleep(0.1)

def read_rgb(): # read colors and list them in red green blue with their respected values ofc
    red = read_word(RED_L)
    green = read_word(GREEN_L)
    blue = read_word(BLUE_L)
    return red, green, blue

def detect_color(r, g, b): # function for certain color detections like orange and blue for the ring
    if r > 15000 and 500 < g < 1500 and b < 800:
        return "orange"
    elif b > 3000 and b > r and b > g:
        return "blue"
    else:
        return "none"

# === Motor Setup single-motor setup for forward/reverse, PWM for speed and it is not omnidirectional and cant go in opposite directions
class Vehicle:
    def __init__(self):
        self.motor = Motor(forward=22, backward=17)
        self.pwm = PWMOutputDevice(27)
        self.set_speed(0.5)

    def set_speed(self, speed): # Func. that changes the speed of the motor
        self.pwm.value = speed

    def forward(self, speed=0.8): # for driving forward
        self.set_speed(speed)
        self.motor.backward()  

    def backward(self, speed=0.8): # backward 
        self.set_speed(speed)
        self.motor.forward()

    def stop(self): # stopping / pausing , depends on usage
        self.motor.stop()
        self.set_speed(0)

# === Main Program ===
init_isl29125() # ISL29125 is the name of our color sensor i figured i just use it as a initiater
rpi = Vehicle() # for the motor start
steer_center()  # for the steering system (servo)

try:
    while True:
        front_distance = get_front_distance() # will calculate distance front and side
        side_distance = get_side_distance()
        print(f"📏 Front: {front_distance} cm | Side: {side_distance} cm")

        r, g, b = read_rgb() # reads the colors and sees values
        color = detect_color(r, g, b)
        print(f"🎨 Detected Color: {color} (R={r}, G={g}, B={b})")

        # === Read from Camera Detection JSON
        label = ""
        try:
            with open("/home/roshdi-24/picamera2/results/detections.json", "r") as f:
                data = json.load(f)
                if isinstance(data, list) and data:
                    label = data[0].get("label", "").lower()
        except Exception as e:
            print("⚠️ JSON read error:", e)

        print(f"📸 Detected Label: {label}")

        # === Object Detection Reactions
        if "redbox" in label:
            print("🔴 Red Box → Turning right")
            steer_right()
            rpi.forward()
            time.sleep(1)
            rpi.stop()
            steer_center()
            continue

        elif "greenbox" in label:
            print("🟢 Green Box → Turning left")
            steer_left()
            rpi.forward()
            time.sleep(1)
            rpi.stop()
            steer_center()
            continue

        elif "xparking" in label:
            print("🅿️ Parking detected. Executing sequence...")
            rpi.forward(0.4)
            while True:
                front = get_front_distance()
                side = get_side_distance()
                if min(front, side) < 8:
                    rpi.stop()
                    print("✅ Parked successfully!")
                    steer_center()
                    pwm.stop()
                    GPIO.cleanup()
                    camera_process.terminate()
                    sys.exit(0)
                time.sleep(0.2)

        # === Obstacle ahead? Time to reverse smartly
        if front_distance < 30: # checks the distance and determines a stop then turn based on side sensor measurements 
            print("⛔ Obstacle! Reversing then choosing direction...")
            rpi.stop()
            time.sleep(0.2)
            rpi.backward(0.6)
            time.sleep(1)
            rpi.stop()

            if side_distance < 15:
                print("🧱 Wall on right → Turn Left")
                steer_left()
            else:
                print("🟢 Right is clear → Turn Right")
                steer_right()

            rpi.forward()
            time.sleep(1)
            rpi.stop()
            steer_center()
            continue

        # === Color-BASED TURNS FOR CLOCKWISE / COUNTERCLOCKWISE
        if color == "orange":
            print("🟠 Orange seen → Turn Right")
            steer_right()
            rpi.forward()
            time.sleep(1)
            rpi.stop()
            steer_center()
            continue

        elif color == "blue":
            print("🔵 Blue seen → Turn Left")
            steer_left()
            rpi.forward()
            time.sleep(1)
            rpi.stop()
            steer_center()
            continue

        # === Default Behavior ===
        print("🚗 Moving forward")
        steer_center()
        rpi.forward()
        time.sleep(0.2)

except KeyboardInterrupt:
    print("🛑 Manual stop. Cleaning up.")
    rpi.stop()
    steer_center()
    pwm.stop()
    GPIO.cleanup()
    camera_process.terminate()
