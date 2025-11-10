"""Gyroscope standstill detection, calibration, and visualization."""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# === PATH CONFIGURATION ===
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE_DIR, "../../..", "data")
DOCS_DIR = os.path.join(BASE_DIR, "../../..", "docs")
RESULTS_DIR = os.path.join(BASE_DIR, "../../..", "results")
CSV = os.path.join(DATA_DIR, "data.csv")
GYRO_CALIB = os.path.join(RESULTS_DIR, "gyro_calib_params.csv")
FIG = os.path.join(DOCS_DIR, "gyro_calibration_results.png")


# === LOAD DATA ===
imu_data = pd.read_csv(CSV).to_numpy()
gyros = imu_data[:, 3:6]


# === EXAMPLE GYROSCOPE CALIBRATION PARAMETERS (adjust as needed) ===
gyr_params = pd.read_csv(GYRO_CALIB).to_numpy().flatten()



# === MATH HELPERS ===
def skew(v):
    """Return the skew-symmetric matrix of a 3D vector."""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


def misalignment(epsilon):
    """Build rotation matrix from small misalignment angles (eq. 5)."""
    return np.eye(3) + skew(epsilon)


def sensor_error_model(errors):
    """Construct sensor error model matrix (M) and bias vector (b)."""
    KX, KY, KZ, NOX, NOY, NOZ, BX, BY, BZ = errors
    M = np.array([
        [1.0 + KX, NOY, NOZ],
        [0.0, 1.0 + KY, NOX],
        [0.0, 0.0, 1.0 + KZ]
    ])
    b = np.array([BX, BY, BZ])
    return M, b


# === GYROSCOPE CALIBRATION ===
def calibrate_gyroscope(measurements, sensor_errors, epsilon):
    """
    Apply gyroscope calibration according to the sensor model (eq. 8).
    """
    M, b = sensor_error_model(sensor_errors[0:9])
    C = np.linalg.inv(M)
    Rm = np.linalg.inv(misalignment(epsilon))

    calibrated = np.copy(measurements)
    for m in calibrated:
        m[:] = C @ Rm @ m - C @ b
    return calibrated


# === STANDSTILL DETECTION ===
def generate_standstill_flags(imu_data):
    """
    Generate binary flags (1 = standstill, 0 = motion)
    based on gyroscope magnitude threshold.
    """
    standstill = np.zeros(len(imu_data))
    counter_after_motion = 60
    cooldown = 60

    for i, m in enumerate(imu_data):
        if np.linalg.norm(m[3:6]) < 0.13:  # gyroscope is quiet
            counter_after_motion += 1
            if counter_after_motion > cooldown:
                standstill[i] = 1
        else:
            standstill[i] = 0
            standstill[i - cooldown:i] = 0
            counter_after_motion = 0
    return standstill


# === PLOTTING ===
def plot_gyroscope_before_and_after(gyros, gyros_calibrated):
    """
    Plot uncalibrated vs. calibrated gyroscope data.
    """
    times = np.arange(0, len(gyros_calibrated)) * 0.01
    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    labels = ['gx', 'gy', 'gz']

    for i in range(3):
        axs[i].plot(times, gyros[:, i], color='red', alpha=0.7,
                    label=f'Uncalibrated ${labels[i]}_{{mean}}$ = {gyros[:, i].mean():.4f} rad/s')
        axs[i].plot(times, gyros_calibrated[:, i], color='green', alpha=0.7,
                    label=f'Calibrated ${labels[i]}_{{mean}}$ = {gyros_calibrated[:, i].mean():.4f} rad/s')
        axs[i].axhline(0, color='blue', linestyle='--', linewidth=2, alpha=0.5,
                       label='Ideal $g_{ideal}$ = 0 rad/s')
        axs[i].grid()
        axs[i].set_ylabel(f'{labels[i]} [rad/s]')
        axs[i].legend(loc='upper right')

    axs[0].set_ylim([-0.03, 0.013])
    axs[1].set_ylim([-0.005, 0.01])
    axs[2].set_ylim([-0.03, 0.017])

    axs[2].set_xlabel('Time [s]')
    fig.suptitle("Gyroscope Standstill Calibration Results", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    #plt.savefig(FIG, dpi=500)
    plt.show()


# === MAIN EXECUTION ===
if __name__ == "__main__":
    standstill_flags = generate_standstill_flags(imu_data)
    gyros = gyros[standstill_flags > 0]
    gyros_calibrated = calibrate_gyroscope(gyros, gyr_params, gyr_params[9:12])
    plot_gyroscope_before_and_after(gyros, gyros_calibrated)
