# Agricultural Rover
![Status](https://img.shields.io/badge/status-in%20development-yellow)
![Platform](https://img.shields.io/badge/platform-ESP32-blue)

An autonomous agricultural rover designed to navigate dirt paths between crop rows and detect plant diseases in real time.

This project combines embedded control, computer vision, and a web-based disease mapping system into a low-cost mobile platform built mainly from accessible off-the-shelf components. The rover is intended for agricultural field scouting, where it can follow crop-row paths, identify diseased plants, and report their estimated position to a remote dashboard.

## Features

- Differential-drive rover with two 37 mm brushed DC motors
- Hall-effect encoder feedback for wheel speed and distance estimation
- ESP32-based real-time motor control over Wi-Fi
- Smartphone-based HSV path following using onboard camera input
- Smartphone-based plant disease detection using a machine learning model
- Web dashboard for mapping detections by row and column
- Low-cost mechanical structure built from plywood.
- Protected power distribution with buck conversion, fuses, and EMI suppression

## System Overview

The rover is built around three main subsystems:

1. **Embedded control**
   - ESP32 generates PWM commands for two BTS7960 motor drivers
   - Reads encoder feedback from both motors
   - Executes motion commands received over Wi-Fi

2. **Vision and intelligence**
   - A smartphone mounted on the rover processes camera frames
   - HSV filtering is used to detect the dirt path between crop rows
   - A machine learning model analyzes plants for disease symptoms

3. **Field mapping**
   - Disease detections are sent to a remote web server
   - A web dashboard displays detections on a field map using row/column coordinates

## Mechanical Design

The chassis is made from plywood with approximate dimensions of 250 mm × 180 mm × 100 mm. It supports the electronics, motors, battery, and smartphone mount while staying lightweight and easy to machine.

The rover uses:
- Two 120 mm diameter off-road RC tires at the front, mounted on custom machined aluminum rims with a D-flat bore matched directly to the motor shafts
- Two matching rear wheels mounted on a passive 6 mm steel axle passing through the chassis

## Electronics

### Motors and drive
- 2 × 37 mm DC gearmotors
- Rated stall torque: 12 kgf·cm each
- Hall-effect encoders for speed and distance feedback
- Differential-drive configuration

### Motor drivers
- 2 × BTS7960 H-bridge motor drivers
- RPWM and LPWM used for speed and direction control
- R_EN and L_EN tied permanently to 5 V

### Logic level conversion
- ESP32 operates at 3.3 V logic
- BTS7960 control inputs require 5 V logic
- A 4-channel level shifter converts the four PWM lines from 3.3 V to 5 V

### Power system
- 3S LiPo battery, nominal 11.1 V, 12.6 V fully charged
- Main power switch for whole-system control
- WAGO-based power distribution
- Four fused branches for protection
- XL4015 buck converter for 5 V rail generation
- 100 nF ceramic capacitors across motor terminals for EMI suppression
- 1000 µF electrolytic capacitors on 12 V and 5 V rails for voltage stabilization

## Software Architecture

### ESP32 firmware
The ESP32 is responsible for:
- Receiving motion commands over Wi-Fi
- Driving both motors using PWM
- Reading wheel encoders
- Supporting speed and position estimation

### Smartphone application
The onboard smartphone performs:
- HSV-based path detection for row navigation
- Steering correction generation
- Real-time disease classification from camera frames
- Communication with the ESP32 and remote server

### Web dashboard
The web interface is used to:
- Receive disease events from the phone
- Display plant disease locations on a row/column field map
- Help the user identify affected areas for targeted treatment

## Applications

This rover is designed as a low-cost platform for:
- Automated crop scouting
- Disease monitoring
- Smart farming research
- Mechatronics and robotics education

## Repository Structure

```text
docs/       Project documentation, diagrams, and images
hardware/   Mechanical files, electronics files, drawings, and BOM
firmware/   ESP32 motor control and encoder code
mobile/     Smartphone vision and disease detection code
dashboard/  Web-based field mapping interface
results/    Test photos, videos, and performance notes
```

## Current Status

This project is currently in development. A two motor prototype has been finished however is not being tested yet due to rims not being machined yet and a faulty second BTS7960.

## Future Improvements

NONE PLANNED.

## Author

Mehdi Djaoud  
L3 MCIL Mechatronics Student
University of Boumerdes