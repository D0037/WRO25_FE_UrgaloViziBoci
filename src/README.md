# Code Module Documentation 
## - WRO25_FE_Úrgaló Vizí Bóczy -

Below you will find a README-style description for each source file/module in the project, describing its purpose, functionality, and connection to the electromechanical parts of the robot. 

---
## 1. main.py
### 1.1 Description:
Main entry point for launching the robot's self-driving behavior.
This file initializes all core modules (image processing, movement), reads the initial position from camera data, and performs a simple maneuver using hard-coded paths.

### 1.2 Key Functions:
- Starts a thread for `image_processing.image_thread`.
- Sets the initial robot position based on colored line detection.
- Executes a sequence of movements (via `move.move` and `move.turn`).

### 1.3 Hardware Links:
- Calls movement commands using motor and servo.
- Uses gyro for heading correction.
- Uses a camera for line detection.


## 2. rc.py
### 2.1 Description:
Provides remote control capabilities via HTTP GET requests (typically from a dashboard or joystick interface). This was only used for testing and not for the competition.

### 2.2 Key Functions:
- Retrieves steering and motor speed from a URL.
- Updates the robot's motor and steering using values from the JSON response.

### 2.3 Hardware Links:
- Directly modifies motor and servo PWM values.
- Uses GPIO pins for motor control enable/disable.


## 3. test.py
### 3.1 Description:
Minimal test script to validate turning function independently. And of course all the other functionalities of the robot.

### 3.2 Key Functions:
- Initializes motors and gyro.
- Runs custom tests for debugging

### 3.3 Hardware Links:
- Motor controller and gyro sensor via GPIO/I2C.


## 4. utils/image_processing.py`
### 4.1 Description:
Handles camera input, color-based line detection, and initial position classification.
Runs in a separate thread to continuously update global state variables.

### 4.2 Key Functions:
- Detects blue and orange lines using HSV masking.
- Calculates line intersection point.
- Classifies start position into one of six pre-defined areas.

### 4.3 Hardware Links:
- Uses a USB camera.
- Provides data for decision-making logic in the robot's navigation.


## 5. utils/movement.py
### 5.1 Description:
Contains all robot motion logic, including:
- Forward motion with gyroscope correction.
- Turning with radius correction.
- Angle and speed settings.

### 5.2 Key Functions:
- `move()`: linear movement with PID-style heading correction.
- `turn()`: turning along a circular path based on radius using PID correction.
- `init()`, `cleanup()`: hardware init/reset for motors and sensors.

### 5.3 Hardware Links:
- Motor controller via GPIO and software PWM.
- Servo motor for front steering with hardware PWM.
- Uses `PositionTracker` for closed-loop motion control.


## 6. utils/tracking.cpp
### 6.1 Description:
C++ code wrapped with pybind11 for high-performance gyroscope data acquisition and mouse-based tracking.
Used as a backend to the `PositionTracker` in Python.

### 6.2 Key Functions:
- Reads gyroscope angular velocity data over I2C.
- Calibrates sensor using multiple samples.
- Continuously integrates values to update angle.

### 6.3 Hardware Links:
- MPU6500 gyro/accelerometer via I2C.
- Optical USB mouse via Linux's /dev/input/mice interface.

## 7. utils/stream.py
### 7.1 Description:
Debugging tool for displaying image streams over HTTP from the robot's onboard camera.
Used during development to verify image processing functionality visually.

### 7.2 Key Functions:
- Stream images continuously using a Flask HTTP server.

### 7.3 Hardware Links:
- USB camera only (software debug tool).


## 8. requirements.txt
### 8.1 Description:
Lists all Python packages required to run the project.

### 8.2 Packages:
- `opencv-contrib-python`: computer vision tasks.
- `numpy`: mathematical operations.
- `RPi.GPIO`: control of GPIO pins for motors.
- `rpi-hardware-pwm`: access hardware-level PWM on Raspberry Pi.
- ect. some more not as important az the aboves.

---
# Maintained and presented by Team D0037 - Úgrálo Vizí Bóczy
Hungary — WRO 2025 Future Engineers Division — 2025.06.25.