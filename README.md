# Two-Axis IMU Calibration Test Stand with Differential Gear System
An open-source, dual-axis calibration platform for IMU sensors (e.g., MPU6050), featuring a fully 3D-printed differential gear drive and automated motion control for precise in-situ calibration.

## Project Overview

This project presents the design and function of a **two-axis IMU calibration test stand**, developed to calibrate inertial measurement units (IMUs) such as the [MPU6050](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/). The test stand enables controlled, repeatable motion for precise sensor calibration.

All mechanical parts are **3D printed**, and motion is driven by **NEMA 17 stepper motors** in combination with an **HTD belt system**. The movement is realized through a **differential gear mechanism**, providing two degrees of freedom (DoF).

The Calibration is based on the method proposed in the following paper:
> **A. Mikov, S. Reginya, and A. Moschevikin**,
> *"In-situ Gyroscope Calibration Based on Accelerometer Data,"*
> 27th Saint Petersburg International Conference on Integrated Navigation Systems (ICINS), IEEE, 2020.
> [[IEEE paaper]](https://ieeexplore.ieee.org/document/9133804) and Github Repository [[imu-calib]](https://github.com/mikoff/imu-calib).


---

## Mechanical Design and Kinematics
Below are the differential gear animations showing the independent Z-axis rotation and combined X/Z-axis motion.
<b>Z-Axis Rotation</b> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <b>X/Y-Axis Rotation</b><br>
<img src="./docs/rotation_z.gif" width="400"> <img src="./docs/rotation_xz.gif" width="400">

<b>Exploded Assembly View</b><br>
<img src="./docs/ExplodedView_AssemblyAnimation.gif" width="600">


---

## Wiring Diagram
The wiring connects two NEMA 17 stepper drivers, an ESP32 microcontroller, and the MPU6050 IMU sensor.
![Wiring Diagram Explanation](/docs/wiring_diagram.png)

---

## Validations
Each calibration result was obtained through a series of standstill measurements and controlled rotations performed by the test stand.
### Accelerometer Validation
![Accelerometer Validation](/docs/acc_calibration_results.png)

### Gyroscope Validation
![Gyroscope Validation](/docs/gyro_calibration_results.png)

### Live Validation
![Live Validation](/docs/live_validation.gif)
---

## Setting Up the Test Stand
Eventually you have to set the correct COM port in the [main GUI](software/python/src/gui/main.py) before running it.
### Prerequisites
```bash
# Clone the repository
git clone https://github.com/anh-lxn/teststand-imu.git
cd teststand-imu

# Create and activate a virtual environment
python -m venv software/python/venv
source software/python/venv/bin/activate   # (Linux/macOS)
# .\software\python\venv\Scripts\activate  # (Windows)

# Install required dependencies
pip install -r software/python/requirements.txt
```

### Compiling and Uploading Firmware to ESP32
The software C++ File for the ESP32 is located under: [cpp folder](software/cpp/)
To build and upload the firmware, you need PlatformIO installed in your IDE (e.g., VSCode).

```bash
# Build and upload the firmware to the ESP32
pio run --target upload

# (Optional) Open the serial monitor
pio device monitor
```

Please note that it takes a few seconds for the ESP32 to boot up after uploading the firmware. (For me its around 30 seconds)

### Running the Graphical Interface
```bash
# Run the main GUI application
python software/python/src/gui/main.py
```
![MAIN GUI](/docs/gui.png)

Buttons:
- **Start Calibration**: The Imu will collect Data in synchronization with the Test Stand Movement. Then it will perform the Calibration and show the Results and save them in the results folder.
- **Read raw data**: The Imu will send raw data to the GUI for live visualization without performing calibration.
- **Stop reading**: Stops any ongoing data reading from the Imu.

### Running Validation Scripts
```bash
# Run accelerometer validation
python software/python/src/accel_validation.py
```

```bash
# Run gyroscope standstill validation
python software/python/src/gyro_standstill_validation.py
```
```bash
# Run live validation
python software/python/src/live_validation.py
```
---


## Authors and Contributors

- **[Anh Le Xuan](https://www.anhlexuan.com)** – Student Assistant
  Designed, built, and programmed the complete IMU test stand, including 3D modeling, electronics, firmware, and software.

- **[Dana Lenzig](https://www.linkedin.com/in/dana-lenzig-61a04830a)** – Student Assistant
  Developed the graphical user interface (GUI) for live visualization and calibration control.

- **Benjamin Waschilewski** – Research Associate and Project Supervisor
- **Hung Le Xuan** – Head of Research Group

*Institute of Textile Machinery and High Performance Material Technology (ITM), TU Dresden*

### Citation

If you use this project or parts of it, please give credit to the following authors:

> **Anh Le Xuan**, **Dana Lenzig**, **Benjamin Waschilewski**, **Hung Le Xuan**
> *Institute of Textile Machinery and High Performance Material Technology (ITM), TU Dresden*
>
> Based on the calibration method described in:
> *Mikov, A., Reginya, S., & Moschevikin, A. (2020). In-situ Gyroscope Calibration Based on Accelerometer Data.
> 2020 27th Saint Petersburg International Conference on Integrated Navigation Systems (ICINS), IEEE.*


---

## License
This project is released under the MIT License.
See [LICENSE](./LICENSE) for details.