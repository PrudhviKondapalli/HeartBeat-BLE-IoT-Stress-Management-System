# HeartBeat-BLE-IoT-Stress-Management-System
HeartBeat BLE is an IoT-based stress management solution developed as part of the ECEN 5823: IoT Embedded Firmware course at the University of Colorado Boulder. This system provides real-time heart rate and blood oxygen level monitoring using Bluetooth Low Energy (BLE) technology.
![image](https://github.com/user-attachments/assets/94b6c502-63ca-4f99-91d6-1ac1b284e104)
# The core components of the system include:
Two EFR32BG13 Blue Gecko boards (one acting as a GATT Server, the other as a GATT Client)
MAX30105 Particle and Pulse Ox Sensor
CAP-1203 Capacitive Touch Slider
COM-09938 10-row LED array
# Features
Real-time heart rate and blood oxygen level monitoring
Visual heart rate feedback via LED array
User interaction through a capacitive touch slider
Low power consumption with multiple energy modes
Secure BLE communication between client and server devices
# Hardware Setup
## GATT Server (Blue Gecko Board 1)
Hosts the MAX30105 sensor for heart rate and blood oxygen measurements
Controls the COM-09938 LED array for visual heart rate feedback
Implements custom GATT services for heart rate, blood oxygen, and LED bar graph data
## GATT Client (Blue Gecko Board 2)
Integrates the CAP-1203 capacitive touch slider for user input
Displays sensor data on its LCD screen
Controls the energy modes and data acquisition from the server
# Software Architecture
The system is built on a client-server model using BLE communication:
![image](https://github.com/user-attachments/assets/69f0a7c9-e070-46d7-bf4e-7b5591daff29)
## GATT Server:
Reads sensor data from MAX30105
Updates LED array based on heart rate
Implements custom GATT services
Manages power modes based on client indications
## GATT Client:
Handles user input via capacitive touch slider
Discovers and connects to server services
Displays sensor data on LCD
Controls server's energy modes and data acquisition
![image](https://github.com/user-attachments/assets/61b73c96-e236-4537-b499-151f7421a9cf)
# Key Features
Energy Efficiency: Utilizes various energy modes (EM0, EM1, EM2) to optimize power consumption.
Secure Communication: Implements encrypted BLE links with bonding between client and server.
User Interface: Capacitive touch slider allows intuitive control of the system.
Visual Feedback: LED array provides an easy-to-understand visual representation of heart rate.
# Getting Started
## Prerequisites
Two EFR32BG13 Blue Gecko boards
Simplicity Studio IDE
MAX30105 Particle and Pulse Ox Sensor
CAP-1203 Capacitive Touch Slider
COM-09938 10-row LED array

## Installation
Clone this repository
Open the project in Simplicity Studio
Flash the server code to one Blue Gecko board and the client code to the other
Connect the hardware components as described in the Hardware Setup section

## Usage
Power on both devices
The client device will automatically scan for and connect to the server
Use the capacitive touch slider to control the system:
Middle segment: Enable/disable heart rate sensor
Left segment: Display heart rate on server LCD
Right segment: Display blood oxygen level on server LCD

# Contributors
Prudhvi Kondapalli (prudhvi.kondapalli@colorado.edu)
Sonal Tamrakar (sonal.tamrakar@colorado.edu)

# Acknowledgments
This project was developed as part of the ECEN 5823: IoT Embedded Firmware course at the University of Colorado Boulder.
