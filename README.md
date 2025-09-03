# 🤖 Autonomous Robot for WRO 2025 - Future Engineers

Welcome to the official repository for our **Autonomous Robot** project, built for the **WRO 2025 Future Engineers Competition**.  

This project demonstrates a **self-driving robot** capable of:  
- Real-time color detection  
- Obstacle avoidance  
- Intelligent navigation  

It is designed to perform reliably in a **structured competition environment**.  

---

## 🏁 Project Overview

Our robot operates autonomously using a combination of **vision** and **ultrasonic sensing**. Its main capabilities include:  

- **Obstacle Detection**  
  - Detects objects in front and sides to prevent collisions.  

- **Box Color Recognition**  
  - 🔴 **Red Box** – avoid and steer to the **right** side  
  - 🟢 **Green Box** – avoid and steer to the **left** side  

- **Real-Time Navigation**  
  - Combines camera input with ultrasonic sensor data for **split-second decision-making**  

- **Flexible Steering Control**  
  - Servo-controlled steering allows **fine adjustments** and smooth turns  

- **Adaptive Speed Control**  
  - Motor driver settings allow the robot to **adjust speed** based on environment and detected obstacles  

> This setup ensures **efficient lap completion, safe navigation, and compliance with WRO 2025 regulations**.  

---

## ⚙️ Technologies & Components

### 🧩 Hardware
- **Matrix Pi 4 Model B** – Chasis; all chasis parts and base come from this kit
  <img width="712" height="488" alt="image" src="https://github.com/user-attachments/assets/bfa36531-b5a3-4e0b-aacb-bbf7418e51ba" />
  <img width="898" height="457" alt="image" src="https://github.com/user-attachments/assets/651e5dfb-c168-4f54-a567-538f5b456849" />


- **Raspberry Pi 4 Model B** – Main processing unit; runs Python code for vision and sensor integration
  <img width="800" height="800" alt="image" src="https://github.com/user-attachments/assets/68347c52-910b-4d98-8b58-7c9e420d184c" />
- **IMX500 AI Camera** – Captures the environment in real-time for **color-based detection** of red and green zones
  <img width="3000" height="1927" alt="image" src="https://github.com/user-attachments/assets/9dc050f5-25ac-4d96-b52d-e67c7c0f5ec0" />
- **L298N Motor Driver** – Controls the robot’s **DC motor speed and direction**
  <img width="2448" height="2138" alt="image" src="https://github.com/user-attachments/assets/e416f76b-b5c2-434b-9d48-0efdffc2f62e" />
  
- **MG995 Servo Motor (Steering)** – Enables **precise turning**; steering angles can be easily configured
    <img width="480" height="431" alt="image" src="https://github.com/user-attachments/assets/f82fc28f-04a1-47f8-8ab2-62ca0afe7c18" />

- **Ultrasonic Sensors (3x)** – Measure distance to obstacles (front, left, right) for **collision avoidance**
    
- **Battery Pack (2x) / Power Bank** – Provides **stable power** to motors and Raspberry Pi  
    <img width="2592" height="2284" alt="image" src="https://github.com/user-attachments/assets/35b07ca5-34b8-4691-9ccd-c3e14c3b5b9a" />
    <img width="519" height="519" alt="image" src="https://github.com/user-attachments/assets/0fa2cc78-1618-40dc-8fbf-aa7d60f94f89" />


### 💻 Software

- **Python** – Primary language for sensor control, vision, and decision-making  
- **OpenCV** – Processes camera frames, detects colored zones using **HSV thresholds**, and identifies contours  
- **pigpio** – Controls PWM for **servo steering** with accurate pulse-width modulation  
- **RPi.GPIO** – Handles digital input/output for **motors, buttons, and ultrasonic sensors**  

---

## 🛠️ How It Works

- **Camera Detection**  
  - The IMX500 camera continuously captures frames  
  - **OpenCV** and HSV thresholds detect **red and green zones**  
  - Each detected zone is logged in a **JSON file**  

- **Obstacle Avoidance**  
  - Three ultrasonic sensors measure distances to nearby objects  
  - Robot adjusts **steering or backs up** based on obstacle proximity  

- **Decision-Making Logic**  
  - Combines inputs from camera and ultrasonic sensors to decide:  
    - **Turn left or right** based on box color or lap direction  
    - **Lock behaviors** for a short duration to prevent rapid switching  
  - Steering is handled by a **servo motor**, with configurable center angle and offsets for left/right turns  

- **Start/Pause Button**  
  - The robot **starts or pauses** when a push-button is pressed, allowing control without a monitor  


