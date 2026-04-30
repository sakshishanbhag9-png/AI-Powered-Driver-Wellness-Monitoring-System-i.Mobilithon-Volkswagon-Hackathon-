# 🚗 AI-Enhanced Driver Wellness Monitoring System  
### Multi-Modal, Privacy-First AI Co-Pilot for Safer Driving

---

## 📌 Overview

Driver fatigue, distraction, and sudden medical emergencies are major causes of road accidents worldwide. Traditional safety systems are either reactive (like airbags) or rely on a single data source (like vision-only alerts), which limits their effectiveness.

This project introduces a multi-modal AI-powered system that continuously monitors the driver using three key inputs:

- Vision: Face, eye state, gaze, and yawning detection  
- Behavior: Steering activity monitoring  
- Physiological signals: Heart rate and SpO₂  

All processing is performed on-device, ensuring privacy, low latency, and offline functionality.

---

## 🚀 Key Features

### AI-Based Detection
- Detects whether eyes are open or closed  
- Uses PERCLOS and EAR for blink and fatigue detection  
- Identifies yawning using mouth aspect ratio (MAR)  
- Tracks head pose and gaze direction  
- Supports multi-class drowsiness detection (not just binary)

### Multi-Modal Fusion
- Combines vision, behavior, and physiological data  
- Detects critical cases missed by single-input systems  
- Improves accuracy and reduces false alerts  

### Intelligent Alert System
- Multi-level alert mechanism:
  - Audio alerts  
  - Voice prompts  
  - SMS alerts using Twilio API  
- Emergency escalation with GPS location sharing  

### Vehicle Safety Features
- Gradual vehicle slowdown during emergencies  
- Automatic vehicle halt system  
- Steering inactivity detection as a fail-safe  

### Privacy-First Design
- Runs entirely on-device (Raspberry Pi)  
- No cloud dependency  
- Ensures user data privacy  

---

## 🧪 Tech Stack

### Software
- Python  
- OpenCV  
- MediaPipe  
- TensorFlow Lite  
- Tkinter (GUI)  
- PySerial  
- Requests  

### Hardware
- Raspberry Pi 4  
- Pi Camera Module  
- ESP32  
- GPS Module (Neo-6M)  
- Motor Driver (L298N)  
- Potentiometer (for steering simulation)  
- Buzzer and LED indicators  

---

## 📂 Project Structure

- main_controller.py → Core AI logic (vision + vitals + control)  
- main_controller.ino → ESP32 vehicle controller  
- assets/ → Audio files and resources  
- docs/ → Reports and documentation  
- requirements.txt → Python dependencies  

---

## ⚙️ Installation & Setup

### Clone the Repository
```bash
git clone https://github.com/https://github.com/sakshishanbhag9-png/AI-Powered-Driver-Wellness-Monitoring-System-i.Mobilithon-Volkswagon-Hackathon
cd driver-wellness-ai

---

## ▶️ How It Works
The camera captures real-time video of the driver
AI models analyze facial features to detect fatigue indicators:
Eye closure
Yawning
Head movement
Steering behavior and vitals data are collected
A fusion engine evaluates the driver’s condition
Alerts are triggered based on severity
In critical situations, the system slows down or stops the vehicle and sends emergency alerts
