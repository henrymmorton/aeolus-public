# AEOLUS Autonomous Runner Pacing Vehicle

This repository holds the brains for the AEOLUS autonomous runner pacing vehicle.

**Overview**

AEOLUS uses a neural network–based computer vision pipeline together with data from a suite of sensors — including a tachometer, GPS, IMU, and ultrasonic distance sensor — to locate itself and provide pacing assistance for its runner (user) along a predetermined path.

**Project Structure**

Explore the different components of the system in the following directories and files:

**High-Level Logic**
- `autonomy.py`
- `autonomy_processes.py`

**Computer Vision**
- `cv/neural_net/vision_pipeline`

**State Estimation**
- `se`

**High-Level Control Systems**
- `sc`

**Microcontroller Code**
- Low-Level Control: `tnsy/control`  
- Sensing: `tnsy/sensors`

**Compute Architecture**

AEOLUS is designed to run on a CUDA-compatible machine paired with a microcontroller.

The current implementation uses:
- Jetson Orin Nano — for inference, state estimation, path planning, and high-level control.
- Teensyduino microcontroller — for sensor measurement, low-level control, and actuation.

Communication between the Jetson and the Teensy is handled via serial communication.

**System Architecture**

<p align="center">
  <img src="assets/glass_box.epng" alt="System Architecture" width="1000">
</p>