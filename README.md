# Mission Overview

## - WRO25_FE_Úrgaló Vizí Bóczy -

## 1. INTRODUCTION

### 1.1 Introduction of the Team

**Team name**: FRT Úgrálo Vizí Bóczy  
**Team code**: D0037  
**School**: Budapesti Fazekas Mihály Gimnázium, Budapest

**Team members**:
- **Serényi Márton** – 9th grade
    - Responsibilities: Software and Hardware development
    - Fun fact: Built an audio amplifier at home out of boredom 

- **Szabó-Komoróczki Csenge Veronika** – 10th grade
    - Responsibilities: Documentation and Data Visualization
	- Fun fact: Loves the cat-themed VSCode color scheme

**Team mentor**: 
- **Sásdi Mariann** -  Informatics teacher
	- Responsible for communication with competition organizers
	- Loved by her students, and always up for a slide trip with them

---

### 1.2 Mission Objectives

The goal of our project is to design and build an autonomous miniature vehicle and participate in the WRO Future Engineers 2025 competition. We aim to perform well in the contest while learning as much as possible and expanding our personal and technical horizons.

The robot's uniqueness lies in its construction: we used a mix of LEGO pieces, hot glue mounting, and 3D-printed components. Although we use a fairly typical set of sensors, a standout feature is the use of a real computer mouse mounted underneath the car for position tracking—an unusual but effective odometry solution.

Our mission will be considered successful if:

- The car completes the required autonomous driving tasks (e.g. 30 seconds of continuous navigation),
- We gain valuable knowledge and experience,
- And most importantly, we have fun and make great memories along the way.
  
---

## 2. CAR DESCRIPTION

### 2.1 Overview 

The Úrgálo Vizí Bóczy robot’s primary mission is to autonomously complete two distinct types of driving challenges, each requiring different navigation strategies and sensing capabilities:

#### Challenge 1 – Open Track

This is the more complex part of the task, where the robot must determine its initial location and orientation on the track. 

**The approach involves:**
	- Using computer vision to detect and mask orange and blue lines via HSV filtering.
	- Determining the intersection point of the two lines.
	- Comparing the car’s position relative to this intersection to calculate its starting quadrant and heading.

A pre-calibrated coordinate system is used, with origin defined as the intersection point of the blue and yellow lines. These calibration values are stored and visualized in a Jupyter Notebook. After determining its position, the robot uses onboard sensors (mouse sensor and gyroscope) to track motion and navigate the track using pre-programmed logic that includes 90-degree turns and 1-1.3m roads to complete 3 full laps calculated outside the inner wall's extendece region.

### Challenge 2 – Obstacle Course

This challenge begins with the robot exiting the designated parking spot using pre-programmed logic. Immediately after, it analyzes the first segment to check whether an obstacle is present. If an obstacle is detected, the robot performs an avoidance maneuver accordingly.

As the robot progresses through the course, it enters each corner segment either from the wall side or the center side. The entry side depends on the color of the previous segment’s last detected obstacle.

There are two distinct navigation strategies based on the entry position:
- **Wall-side Entry**:
    - The robot performs a 90° turn in the direction of travel.
    - It calibrates the obstacle avoidance algorithm for the upcoming segment.
    - Then, it reverses almost fully to the adjacent wall.
    - Finally, it executes the avoidance maneuver based on the calibrated data.

- **Center-side Entry**:
    - The robot drives forward toward the opposite wall.
    - Then it performs a reverse-turn to align with the opposite side wall.
    - After a partial reverse (stopping at the midpoint), it calibrates the upcoming avoidance algorithm.
    - It then proceeds to the far wall and performs the final obstacle avoidance maneuver.

This dynamic path planning allows the robot to adjust its behavior in real-time based on previous segment data. The precise turning, reversing, and segment-specific strategies are detailed in the source code and were optimized through extensive testing. Obstacle avoidance and parking techniques are detailed further in the source code documentation.

#### System Components

| Component        | Function                  | Model                                        | Price | Source          |
| ---------------- | ------------------------- | -------------------------------------------- | ----- | --------------- |
| Raspberry Pi     | Main Controller           | Raspberry Pi 5                               | $84   | raspberrypi.com |
| Motor Controller | H-bridge motor controller | BTS7960-M                                    | $6    | Hestore         |
| Camera           | Vision                    | Generic USB Camera                           | -     | Recycled        |
| Gyroscope        | Orientation Sensing       | MPU-6500                                     | $2    | Hestore         |
| Mouse Sensor     | Dead Reckoning            | PAW3515DB                                    | ~$1.5 | Recycled        |
| DC Motor         | Movement                  | Maverick Strada MT, XT, XB, SC, 499 LE motor | ~$10  | Recycled        |
| Servo            | Steering                  | Generic Servo                                | ~$2   | Recycled        |
| Battery          | Power Source              | 7.4V 1000mAh Li-Po                           | $10   | RC hobby store  |
| Buck Converter   | Voltage Regulation        | HW-384                                       | $1    | Hestore         |
| Frame/Wheels     | Mechanical Base           | Recycled RC Car                              | -     | Recycled        |
  
**Other Mechanical Structure**:
- 3D-printed mounting plate
- LEGO bricks and 3D-designed LEGO-compatible gap fillers
- Power switch and separate start button

##### Aditional reasoning
 
 **Raspberry Pi 5**: 
- High-performance SBC with Linux OS support, camera input, USB, GPIO, and multithreading support

**Motor Controller - BTS7960-M**:
- Handles up to 43A; provides reliable motor control with PWM input from Raspberry Pi

**USB Camera - Generic**:
- Plug-and-play support with Raspberry Pi, compatible with OpenCV

**Gyroscope – MPU-6500**:
- Affordable, widely supported I²C gyroscope with good accuracy

**Optical Mouse Sensor – PAW3515DB**:
- Provides inexpensive odometry; offers high-precision position change detection

**DC Motors – Maverick Strada**:
- Reliable torque, fits existing toy car chassis

**Generic Servo**:
- Simple PWM control, lightweight, compatible with RC-grade mechanical mounts

**Note**:  <u>Datasheets are available in the /others folder.</u>

### 2.2 Mechanical / Structural Design

The chassis of the UrgaloViziBoci robot is based on a recycled RC car frame that was modified to meet the requirements of the WRO Future Engineers 2025 challenge.

#### Structural 3D Elements:
  
**Sensor and MCU Platform**: 
  - Custom-designed 3D-printed plate mounted on top of the chassis. Its main function is to expand the usable surface area for mounting components.
  
**Camera Support Column**: 
  - Constructed from LEGO bricks, hot-glued together to provide a stable structure for the webcam.

#### Mounting Methods:

**Screws**:
 - Used to mount the mouse sensor underneath the chassis using a glass jar lid and three screws to ensure a stable and aligned position.

**Hot Glue**:
- Applied to secure the LEGO-based camera tower and the start button.

**Double-Sided Industrial Tape**:
- Used to attach the Raspberry Pi 5, motor controller, voltage converter, gyroscope, and the main power switch.

**Battery**:
- Fits snugly into its designated slot without requiring additional fixation.

**Motors and Servo**:
- Integrated directly into the RC car frame as part of the original structure.

#### Backup Parts and Reliability:

To ensure continuous operation during the competition, the team is prepared with:

- Four fully charged batteries (sufficient for approx. 1 hour runtime).
- Backup modules for all electrical and structural components (except the base frame).
- Full toolkit for on-site repairs and modifications.

#### Visual Documentation:

- <u>System schematic</u>: 
  Located in the   `other/` folder.
  
- <u>Photos of the robot</u>: 
  Available in the  `foto/` directory.
  
- <u>Videos of the robot and the movement</u>: 
  Reachable in the `` video/``  folder.

### 2.3 Electrical Design

The electrical system of the Úrgálo Vizí Bóczy robot is built around a Raspberry Pi 5 controller and powered by RC-grade Li-Polymer batteries. The components are wired and organized to ensure reliable power distribution and compact integration with the chassis.

#### Power System:

**Battery - Li-Polymer (Li-Po), 7.4V, 1000mAh**:
- Lightweight, high current capacity, widely used in RC applications.
**Buck converter - HW-384**:
- Ensures safe and stable voltage for sensitive components.
**Power Switches**:
- Main Power Switch: Connects and disconnects the battery from the system.
- Start Button: Connected to the Raspberry Pi; initiates code execution when pressed.

#### Communication:

- During development, the Raspberry Pi communicated with a local server via WiFi (for remote control and debugging).
- In competition mode, the robot operates autonomously. The code runs directly on the Raspberry Pi 5 and is triggered by the Start button.

#### Circuit and Wiring:

- <u>Detailed wiring</u>: included in the `other` folder.

### 2.4 Software Design

The software of the Úrgalo Vizí Bóczy robot was written primarily in **Python**, leveraging libraries such as **OpenCV** for real-time computer vision tasks. To achieve low-latency and high-frequency sensor integration, a **C++ backend** was developed specifically for gyroscope processing.

Development was carried out in **Visual Studio Code**, with all modules structured to allow concurrent execution using multi-threading.

#### <u>KEY</u> Software Modules:

1. **Gyroscope Calibration & Integration (C++)**
    - Runs at startup
    - Integrates real-time angular velocity to track orientation
    - Works in parallel with mouse odometry for enhanced position tracking

2. **Position Tracking**
    - Combines data from the mouse sensor and gyroscope
    - Continuously estimates displacement and heading
    - Implemented as a background service

3. **Image Processing**
    - Real-time frame analysis using OpenCV
    - HSV filtering to identify orange and blue lines
    - Computes the intersection point of color-coded lines for localization
    - Obstacle detection based on color zones (for Challenge 2)

4. **Movement Controller**
    - Translates steering and speed commands into PWM signals
    - Enables precise radius-based turning and obstacle maneuvering
    - Reacts to high-level decisions from the main control logic

5. **PID Algorithm**
    - Ensures the robot remains on track during turns
    - Dynamically adjusts steering angles based on deviation from ideal path
    - Tuned to minimize overshoot and oscillation in real-time motion

- <u>Additional explanation</u>: provided in the document located in `src/` and in `other/README.md.
- <u>Note!:</u> That not all of the coding documents are explained here. This section is intended to provide an overview of our coding method and is not to be fully precise or exhaustive.
#### Behavior Flow

**Challenge 1 – Open Track Navigation**:

![Flowchart](https://http.smarci.hu/wro/challenge1_flowchart.svg)   

**Challenge 2 – Obstacle Course**:

![Flowchart](https://http.smarci.hu/wro/challenge2_flowchart.svg)


<u>Note</u>: The software does **not** store or log data for more then a few millisecs; all reactions are made in real-time.  
For further technical insights and code explanations, refer to `src/README.md`.

---

## 3. SUMMARY

### Summary and Final Reflections

The **FRT Úgrálo Vizí Bóczy** team designed and built an innovative autonomous robot to participate in the **WRO Future Engineers 2025** competition. The robot was developed to complete two distinct challenges: open track navigation and obstacle avoidance. The aim of the project extended beyond performance—it was also about learning, collaboration, and applying engineering principles in a hands-on context.

The robot is based on a recycled RC car chassis, enhanced with LEGO structures, 3D-printed components, and industrial mounting techniques. Its control system is powered by a **Raspberry Pi 5**, running a modular software stack built in **Python** and **C++**. One of the most distinctive features is the use of an **optical mouse sensor** for position tracking, combined with a gyroscope to achieve accurate odometry.

Throughout development, the team emphasized modularity, reliability, and practical implementation—from power distribution and sensor integration to software architecture and mechanical stability. Comprehensive documentation, spare parts, and on-site repair tools were prepared to ensure readiness for competition conditions.

---

### Closing Statement

This project was more than a technical challenge—it was a comprehensive learning journey. Each team member deepened their understanding of robotics, programming, computer vision, and systems integration. We learned how to turn ideas into functioning systems and how to treat every failure as a learning opportunity.

Although our initial goal was to create a functional and competitive robot, the greatest achievement lies in the **skills gained, the experiences shared, and the lasting teamwork built along the way**.


---

# Maintained and presented by Team D0037 - Úgrálo Vizí Bóczy
Hungary — WRO 2025 Future Engineers Division — 2025.06.25.


