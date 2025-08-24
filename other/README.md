# Documentation of the testing end debugging process

## - WRO25_FE\_Úrgaló Vizí Bóczy -

This section provides a summary of the key issues we encountered during development and how we resolved them. Each problem is followed by our solution and technical or creative choices that led to a working implementation.

---

## Gyroscope

**Problem:**  
We wanted to communicate with the gyro over SPI, but the single library that supportsthe BNO085 on a Raspberry PI had a broken SPI functionality. And since we have already ordered the custom designed PCBs, we couldn't use I2C.

**Solution:**  
We used Adafruit's Arduino library for the sensor as a base, and rewrote the hardware hardware abstraction layer, to get it working on a Raspberry Pi.

---

## Hardware PWM & Software PWM

**Problem:**  
The Raspberry Pi 5 provides only two hardware PWM channels, which was insufficient for all components (we needed PWM for the servo and both directions of the motor controller).

**Solution:**  
We prioritized precision for the steering servo and assigned it a hardware PWM channel. The DC motors were driven using software PWM, which, while less accurate, was sufficient for propulsion.  
Configuring hardware PWM on the RPi 5 was non-trivial, but after overcoming driver and pin assignment challenges, we achieved a stable setup.

---

# Maintained and presented by Team Úgrálo Vizí Bóczy

Hungary — WRO 2025 Future Engineers Division — 2025.06.25.