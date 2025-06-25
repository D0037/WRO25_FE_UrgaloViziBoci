# Documentation of the testing end debugging process
## -  WRO25_FE_Úrgaló Vizí Bóczy -

This section provides a summary of the key issues we encountered during development and how we resolved them. Each problem is followed by our solution and technical or creative choices that led to a working implementation.

---

## Mouse Sensor + Gyroscope Fusion

**Problem:**  
Precise tracking of the robot’s position was a core requirement from the start. However, a single sensor could not provide both linear displacement and rotational heading.

**Solution:**  
We mounted a disassembled USB optical mouse on the underside of the robot to measure displacement. While this provided accurate linear movement data, it could not detect rotation. To resolve this, we added an MPU6500 gyroscope.  
Since the gyroscope returns angular velocity, we implemented real-time integration (in C++) to calculate absolute rotation angles. At program startup, the gyro is calibrated to compensate for drift.  
Finally, the displacement and angular data were combined mathematically to create a precise absolute position tracking system.

---

## Hardware PWM & Software PWM

**Problem:**  
The Raspberry Pi 5 provides only two hardware PWM channels, which was insufficient for all components (we needed PWM for the servo and both directions of the motor controller).

**Solution:**  
We prioritized precision for the steering servo and assigned it a hardware PWM channel. The DC motors were driven using software PWM, which, while less accurate, was sufficient for propulsion.  
Configuring hardware PWM on the RPi 5 was non-trivial, but after overcoming driver and pin assignment challenges, we achieved a stable setup.

---

## Starting Position Detection via Vision

**Problem:**  
Accurately determining the robot’s starting position and orientation was essential for autonomous navigation, but color-based vision can be noisy and environment-sensitive.

**Solution:**  
We used OpenCV to process the camera feed:
	- Applied Gaussian blur to reduce noise
	- Converted the image to HSV for better color segmentation
	- Generated a grayscale version to detect the top black field boundary and crop out irrelevant background
	- Created color masks to detect orange and blue lines
	- Fitted lines to these contours and computed their intersection point, even when the point lies outside the visible area  
Using data from earlier calibration runs (stored in the `others/` folder), we mapped this intersection to a known field coordinate.  
Direction was estimated from the slopes of the fitted lines. This same approach was later reused to detect vertical marker poles in the second challenge.

---

## Custom Mounting and Physical Integration

**Problem:**  
Several components—especially the camera and mouse sensor—needed stable and precise mounting to work correctly. Off-the-shelf solutions were not always available.

**Solutions:**
	- The camera tower was built from hot-glued LEGO bricks.
	- The mouse was mounted using a metal jar lid and screws.
	- Components like the Raspberry Pi, gyro, and motor controller were fixed with strong double-sided tape.
	- We printed custom LEGO-compatible gap-filler parts to help support irregular surfaces and improve structural integrity.

---

## Field Setup and Mat Issues

**Problem:**  
We needed a competition-style game mat to perform realistic full-track tests, but we lacked official materials.

**Solution:**  
We created our own track using the reverse side of a previously used Robomission field from the school’s robotics club.  
Although lighting and background colors varied in the test room, preprocessing steps (cropping, blurring, and HSV filtering) helped reduce the impact of environmental conditions.  
A photo of the field setup is included in the `hardware/other/` directory.

## Turning algorithm

**Problem:**
One of the fundamental capabilities required for the vehicle is the ability to execute turns. In the context of this project, the primary challenge was not merely enabling the turning motion, but ensuring its precision. 

**Solution:**
Our objective was to make the vehicle follow the desired path with minimal deviation, particularly during turning maneuvers. Applied Method: To achieve this, we implemented continuous real-time monitoring of the vehicle’s orientation. The system tracks the vehicle’s position and heading in the (X, Y) coordinate space and applies corrective adjustments to the motion based on deviations from the desired trajectory. These corrections are computed using a classical PID (Proportional–Integral–Derivative) control algorithm. The PID controller continuously evaluates the error between the current and target positions, and adjusts the steering accordingly to reduce this error over time. This approach enables the vehicle to dynamically align its path with the intended curve, resulting in high-precision turning performance.


# Maintained and presented by Team D0037 - Úgrálo Vizí Bóczy
Hungary — WRO 2025 Future Engineers Division — 2025.06.25.