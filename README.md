# Autonomous Robot for WRO 2025 - Future Engineers

Welcome to the official repository for Team KCST's Autonomous Robot project, developed for the WRO 2025 Future Engineers Competition.  

This robot is capable of autonomous driving, real-time color detection, obstacle avoidance, and intelligent navigation. It demonstrates practical engineering and problem-solving applied directly to competition requirements.

---

## Project Overview

Our robot operates autonomously using a combination of computer vision and ultrasonic sensing to perform 2 challenges. First the Open challenge, where no boxes are put on the track and the robot vehicle is deployed in a specified position that the judges are to choose, once round starts robot should drive straight and steer depending on correct formation whether it's clockwise or counter-clockwise. After finishing 3 laps of this task, it will shutdown and stop completely. The second challenge is where we found many difficulities, the robot should do the same task as the open challenge but now with boxes placed in the specific locations that are colored either (red or green). The robot should behave a certain way depending on the color it detects and steer a certain direction that depends on the color of the box ahead. Finally, once 3 laps of avoiding obstacles and doing laps is done the robot should stop. (OPTIONAL ROUTE) Starting in the parking area and then leaving to laps whilst avoiding colored boxes and in the end after 3 laps, it should go inside the parking area and stop there.

### Main Capabilities:
- Obstacle Detection: Detects objects in front and to the sides to prevent collisions.
- Box Color Recognition:
  - Red Box → avoid and steer to the right side
  - Green Box → avoid and steer to the left side
- Real-Time Navigation: Uses ultrasonic sensor data to make split-second decisions.
- Configurable Steering and Speed: Servo angles and motor speed can be tuned for optimal performance.
- Adaptive Behaviors: The robot locks into a maneuver for a few seconds to avoid erratic switching.
- Start/Pause Button: A single button toggles the robot between paused and running states.

---

## Hardware Components
– Base chassis and mechanical components

  <img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/bfa36531-b5a3-4e0b-aacb-bbf7418e51ba" /> <img width="898" height="457" alt="image" src="https://github.com/user-attachments/assets/651e5dfb-c168-4f54-a567-538f5b456849" />
  
- Raspberry Pi 4 Model B – Main processing unit; runs Python code for vision and sensor integration
  
  <img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/9dc050f5-25ac-4d96-b52d-e67c7c0f5ec0" />
  
- IMX500 AI Camera – Captures real-time video for detecting red and green zones
  
  <img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/68347c52-910b-4d98-8b58-7c9e420d184c" />
  
- L298N Motor Driver – Controls DC motor speed and direction
  
  <img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/e416f76b-b5c2-434b-9d48-0efdffc2f62e" />
  
- MG995 Servo Motor – Provides steering with configurable center and offset angles
  
  <img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/f82fc28f-04a1-47f8-8ab2-62ca0afe7c18" />
  
- Ultrasonic Sensors (x3) – Mounted at front, left, and right for collision avoidance
  
  <img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/4ad22a96-c1a5-42f4-83c0-3bc161f9fe8b" />
  
- Battery Pack (x2) - (Total of 4 batteries used)
  
  <img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/35b07ca5-34b8-4691-9ccd-c3e14c3b5b9a" />
  <img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/e96db546-29f5-474c-b2d8-6da7c1b48929" />

- Power Bank – Separate power supplies for motors and Raspberry Pi
  
  <img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/0fa2cc78-1618-40dc-8fbf-aa7d60f94f89" />

---

## Software Components

- Python – Core programming language for robot control
- OpenCV – Processes camera frames and detects red/green boxes using HSV thresholds
- pigpio – Provides accurate PWM control for servo steering
- RPi.GPIO – Handles input/output for motor driver, ultrasonic sensors, and button control

---

## How It Works

### Camera Detection
The IMX500 camera captures continuous frames which are processed using OpenCV. HSV thresholds are applied to detect red and green objects. Detections are logged to a JSON file with time, position, and detection order.

### Obstacle Avoidance
Three ultrasonic sensors provide distance measurements. If an obstacle is detected within range, the robot steers away or briefly reverses before resuming.

### Decision-Making Logic
Inputs from the camera and sensors are combined to determine behavior:
- Steer right if a red box is detected
- Steer left if a green box is detected
- Lock behaviors for a short duration to prevent rapid switching
Steering is controlled by a servo motor, with adjustable center angle and offsets for left and right.

### Start/Pause Button
A single push button toggles the robot between paused and active states, allowing control without the need for a monitor.

---

## Repository Structure

- `Code` – Main program and testing scripts
- `Schemes` – CAD files, diagrams, journals, and BOM
- `ImagesV` – Photos of prototypes and robot parts
- `ImagesT` – Team photos, logo, and documentation of progress
- `Videos` – YouTube links showing robot trials

---

## Strategy

Our strategy for this year’s challenge focuses on reliability and simplicity:
  We are using ultrasonic sensors as our eyes for direction and a simple machine learning aspect within our code so that it locks the first steering decision as the correct decision for the corner turns, and our timed steering which performs         exceptionally for our case instead of having a case-oriented steering system. Jokes aside, our goal is to simply not crash.

---

## Team

Team Leader: Abbas Modhaffar
Team Members: Sara Salman | Qutiba Almanshad
We are a dedicated group of students competing in WRO 2025 Future Engineers, focused on building practical autonomous systems within limited time and resources. Abbas Which is me, focused on the task of coding and understanding the logic behind our task and how we should design the robot. Sara and Qutiba both worked on the robot structure and design, as well as finding solutions and out of the box methods to make the robot complete these given tasks.

---

## Demo Video

You can find a demonstration of the robot in action here: [[YouTube Video Link](https://youtu.be/r5Aq6oNJvic?si=vGrpnwZhYPXGCZbK)]

---

## Conclusion & Documentation

This robot integrates computer vision, ultrasonic sensing, and embedded control into a compact and configurable platform. It is designed to meet the requirements of WRO 2025 Future Engineers while being flexible enough for continuous testing and improvement.
