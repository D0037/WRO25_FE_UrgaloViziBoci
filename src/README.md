# WRO 2025 Future Engineers Robot – "Úrgálo Vizí Bóczy"

## 1. Introduction

### 1.1 Team Overview

**Team Name**: FRT Vizí Bóczy
**School**: Budapesti Fazekas Mihály Gimnázium, Budapest

**Team Members**:

- **Márton Serényi** – 10th grade 
  - Responsibilities: Software and Hardware Development
  - Fun fact: Built an audio amplifier at home out of boredom
- **Csenge Veronika Szabó-Komoróczki** – 11th grade 
  - Responsibilities: Documentation and Data Visualization
  - Fun fact: Loves the cat-themed VSCode color scheme

**Team Mentor**:

- **Mariann Sásdi** – Informatics Teacher 
  - Responsible for communication with competition organizers
  - Loved by her students, and always up for a trip with them

### 1.2 Mission Objectives

Our project’s goal is to design and build an autonomous miniature car and participate in the **WRO Future Engineers 2025** competition. We aim to complete the required driving challenges while learning advanced robotics, teamwork, and engineering skills.

The robot’s uniqueness lies in its hybrid sensing approach: using both **TOF LIDAR sensors** and a **mouse optical sensor** for odometry and localization. Combined with a Raspberry Pi 5 and a modular software stack, this provides robust and adaptable navigation.

Success will be defined as:

- Completing the autonomous driving challenges reliably.
- Gaining technical knowledge and problem-solving skills.
- Having fun and creating lasting memories as a team.

---

## 2. Car Description

### 2.1 System Overview

The Úrgálo Vizí Bóczy robot is designed to complete two major challenges:

**Challenge 1 – Open Track**:

- Camera detects track lines (orange and blue) using HSV filtering.
- Calculates intersection point to determine initial position and orientation.
- Uses **TOF distance sensors** and **gyroscope** for odometry (mouse sensor not used in this challenge).
- Follows programmed logic of turns and straight sections to complete 3 laps.

**Challenge 2 – Obstacle Course**:

- Starts from parking using predefined logic and the help of the TOF sensors.
- Checks each segment for obstacles via camera + TOF.
- If obstacle detected → avoidance maneuver based on wall-side or center-side entry.
- Uses both **TOF sensors** and **mouse odometry** (for accurate displacement).
- Executes calibrated turns and maneuvers in real-time, adapting to prior segment data.

---

### 2.2 Hardware Components

| Component        | Function                     | Type / Model                                                                               | Mounting / Notes                                                                                           |
|------------------|------------------------------|--------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------|
| **Main Controller**  | Processing Unit              | Raspberry Pi 5                                                                             | Custom PLA-printed frame + custom RPi header PCB                                                           |
| **Motor Controller** | H-Bridge Motor Driver        | [BTS7960-M](https://www.hestore.hu/prod_10038105.html)                                                                                  | Integrated into 3D-printed frame, cooled by dedicated fan                                                  |
| **Camera**           | Computer Vision              | Raspberry Pi Camera Module 3 Wide                                                          | Mounted on 3D-printed tower                                                                                |
| **Gyroscope**        | Orientation Sensing          | [BNO085 Breakout board](https://www.hestore.hu/prod_10044901.html)                                                                      | Connected via custom RPi header PCB                                                                        |
| **Mouse Sensor**     | Odometry                     | Genius smth                                                                                | 3D-printed plate, hot glued to the half disassembled mouse, the plate can slide up and down, on metal rods |
| **DC Motor**         | Drive Power                  | 1× RC motor (Maverick Strada)                                                              | Integrated in original RC chassis                                                                          |
| **Servo**            | Steering                     | RC Servo                                                                                   | Original RC chassis element                                                                                |
| **Battery**          | Power Source                 | KAVAN Li-Po 2200mAh, 11.1V, 40/80C, 24.4Wh                                                 | Mounted in custom printed holder, routed via power distribution PCB and main switch                        |
| **Buck Converters**  | Voltage Regulation           | 2× 5V USB (Raspberry Pi, servo) + 1× [high-current adjustable voltage converter](https://www.hestore.hu/prod_10043537.html)             | Mounted in 3D Printed frame, cooled with small heatsinks; motor converter cooled with 12V 50mm fan         |
| **TOF Sensors**      | Lateral Distance Measurement | 3× VL53L1X Breakout board                                                                  | Mounted laterally into printed frame                                                                       |
| **Voltage Monitor**  | Current/Voltage Sensing      | ADS1115                                                                                    | Connected on custom RPi header PCB                                                                         |
| **Frame + Wheels**   | Mechanical Base              | Modified RC car chassis with 3D-printed add-ons                                            | Sturdy, custom-designed mounts                                                                             |
| **Custom PCBs**      | Power + Sensor Interfaces    | 1× Power distribution PCB (battery, motor, switch) + 1× RPi HAT PCB (sensors, start/reset) | Mounted on 3D-printed frame                                                                                |
| **Decoration**       | Team Mascot                  | 3D-printed cow figure                                                                      | Mounted at the front of the robot                                                                          |

---

### 2.3 Mechanical / Structural Design

- **Base Frame**: Modified RC car chassis with PLA add-ons.
- **3D-printed structures**: Mounts for Pi, converters, sensors, camera, motor controller and battery.
- **Camera tower**: Mounted on 3D-printed column.
- **Mouse sensor**: Under chassis with custom PLA holder.
- **Cooling system**: Motor converter cooled with fan; other converters with heatsinks.
- **Decoration**: Mascot cow 3D-printed on the front.

Mounting is done by M3 screws, on small parts hotglue.

---

### 2.4 Electrical Design

We wanted a clean desing, so we custom designed two PCBs.

1. One handles the power, converts it to 5V for the electronics
2. The second one is a custom Raspberry Pi HAT, that connects the sensors, the servo and the motorcontroller to the RPi.

- **Power**: KAVAN 11.1V Li-Po → distributed via custom power PCB.
- **Buck converters**: 5V for RPi & servo; and a [high-current adjustable voltage converter](https://www.hestore.hu/prod_10043537.html) to supply consistent voltage to the motor.
- **Main switch + buttons**: Easy operation control.
- **Sensors**: Connected via RPi header PCB.
- **Motor controller**: [BTS7960-M](https://www.hestore.hu/prod_10038105.html) H-bridge motor controller

#### 2.4.1 5V buck converters

- <https://www.hestore.hu/prod_10041976.html>
- They step down DC voltage to 5V for the electronics and servo.
- One is connected to the pi, the other one powers the servo. In previous iterations of the robot, the use of a single buck converter lead to it over-heating, causing the Pi to shut down.

#### 2.4.2 High-current buck converter

- Supplies constant voltage to the motor.
- It prevents the battery voltage to affect to runs

#### 2.4.3 Feedback LEDs

- They provide visual feedback.
- They are connected to GPIO pins on the Pi, in series with current limiting resistors.

#### 2.4.3 Buttons

- It provides an easy way of starting and stopping the robot
- They are connected to GPIO pins, which are pulled high internally in the Pi.
- When they are pressed, they connect the GPIO pins to ground, via some 330 Ohm resistors, which are there to prevent shorting Pi by a programming mistake.

#### 2.4.4 Gyroscope

- To select SPI mode one it's PS0 pin has to be pulled high, and a pull down resistor has to be removed from the breakout board

#### 2.4.5 Motor controller

- It is controlled by a PWM signal produced by the Raspberry Pi.
- Two enable pins have to be HIGH to enable both halfs of the H-Bridge driver.
- It has current sense pins, which output current proportional to the motor current. They are connected through 1k resistor ground, which allows as to measure the voltage drop on the resistor with an ADC, and calculate the current flowing through the motor.

(PCB designs and electrical drawing are located in the  `/schemes/` folder.)

---

### 2.5 Software Design

- **Language**: Python (OpenCV, sensor control) + C++ (gyro backend).
- **Modules**: 
  - Position tracking (mouse + gyro fusion) wirtten in C++
  - Image processing (OpenCV)
  - ToF sensor data processing
  - Motion controller (PWM signals for motor + servo)
  - PID-based steering corrections

Some modules run as concurrent threads on Raspberry Pi 5. The robot does not log data long-term; decisions are made in real-time.

**Flowcharts:**

- **System Startup**: Power on → Initialize sensors → Calibration → Ready.
- **Challenge 1 Flow**: Vision detects track → ToF sensors → Follow path → 3 laps → Stop.
- **Challenge 2 Flow**: Exit parking → Obstacle detection → Wall-side/Center-side avoidance logic → Loop until finish.
- **Software Architecture**: Multithreaded modules (Vision, Odometry, Controller) feeding into main decision loop.

(Flowcharts included as images in `/docs/`.)

---

## 3. Competition Backup Strategy

### Spare Parts

- 1× Li-Po Battery (KAVAN 2200mAh 11.1V)
- 1× TOF400C-VL53L1X sensor
- 1× 5V Buck Converter
- 1x Adjustable voltage high-current buck converter (for the motor)
- We also wanted to bring a backup motor controller, but it was sadly needed during tesing.

### Tools and Repair Equipment

- Soldering station
- Hot glue gun
- Full toolbox (pliers, screwdrivers, cutters)
- Extra jumper wires and cables

---

## 4. Summary

The **FRT Vizí Bóczy** team has built an innovative robot for the WRO Future Engineers 2025 challenge. The robot combines **TOF sensors, optical odometry, and vision** to navigate both open tracks and obstacle courses. Built on a recycled RC chassis with custom 3D-printed parts and PCBs, the robot reflects a balance of creativity, engineering, and practical problem-solving.

Our greatest achievement is not just a working robot but the **knowledge gained and teamwork developed** throughout the process.

Odometry is the use of data from motion sensors to estimate change in position over time. It is used in robotics by some legged or wheeled robots to estimate their position relative to a starting location. This method is sensitive to errors due to the integration of velocity measurements over time to give position estimates. Rapid and accurate data collection, instrument calibration, and processing are required in most cases for odometry to be used effectively. [Wikipedia](https://en.wikipedia.org/wiki/Odometry)

---

**Maintained and presented by Team Úgráló Vizíbóczy**\
Hungary — WRO 2025 Future Engineers Division