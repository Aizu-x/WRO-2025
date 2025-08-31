# 🤖 Autonomous Robot for WRO 2025 - Future Engineers

Welcome to the official repository for our **Autonomous Robot** project, built as part of our entry for the **WRO 2025 Future Engineers Competition**. This project showcases a self-driving robot capable of detecting colored zones, avoiding collisions, and intelligently navigating its environment — all under the constraints of time, resources, and technical challenges. (Made within a timeframe of 2 weeks)

## 🏁 Project Overview

Our robot was designed to operate autonomously in a structured environment with the following capabilities:

- **Obstacle Detection** using real-time vision and ultrasonic sensing.
- **Color Zone Recognition**:
  - 🔴 **Red** – Obstacle zone
  - 🟢 **Green** – Obstacle zone
  - 🅿️ **Pink** – Parking area
- **Collision Avoidance** with **two ultrasonic sensors**, enabling the robot to detect nearby objects and avoid crashes during navigation.

This repository contains all the core code used in the competition.

## ⚙️ Technologies & Components

### 🧩 Hardware:
- Microcontroller (e.g., Raspberry Pi 4 Model B)
- 3x Ultrasonic Sensors (for distance measurement and obstacle avoidance)
- IMX500 AI Camera (for color-based zone detection)
- Motor Driver L298N 
- Power Supply (Battery Pack)

### 💻 Software:
- Python (primary language)
- OpenCV (for color detection and image processing)
- YOLO
- Sensor libraries for real-time data handling

## 🛠️ How It Works

- **Color Detection**: The camera continuously scans the environment and uses HSV thresholds to identify red, green, and pink zones including trained data that contains thousands of pictures.
- **Ultrasonic Sensors**: Positioned both in the front sides to provide obstacle distance readings in real-time.
- **Decision-Making Logic**: Combines input from sensors and camera to make real-time navigation decisions (e.g., stop, turn, park).



