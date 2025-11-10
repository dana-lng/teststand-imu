"""Accelerometer validation """

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
ACCEL_CALIB = os.path.join(RESULTS_DIR, "accel_calib_params.csv")
FIG = os.path.join(DOCS_DIR, "acc_calibration_results.png")


# === LOAD DATA ===
imu_data = pd.read_csv(CSV).to_numpy()
accs = imu_data[:, 0:3]


# === ACCELEROMETER CALIBRATION PARAMETERS===
acc_params = pd.read_csv(ACCEL_CALIB).to_numpy().flatten()

# === SENSOR MODEL ===
def sensor_error_model(errors):
    """
    Construct the sensor error model matrix (M) and bias vector (b)
    according to eq. (1) and (2) from the calibration model.
    """
    KX, KY, KZ, NOX, NOY, NOZ, BX, BY, BZ = errors
    M = np.array([
        [1.0 + KX, NOY, NOZ],
        [0.0, 1.0 + KY, NOX],
        [0.0, 0.0, 1.0 + KZ]
    ])
    b = np.array([BX, BY, BZ])
    return M, b


# === ACCELEROMETER CALIBRATION ===
def calibrate_accelerometer(measurements, sensor_errors):
    """
    Apply calibration to accelerometer measurements.
    See eq. (7) in the referenced calibration paper.
    """
    M, b = sensor_error_model(sensor_errors)
    C = np.linalg.inv(M)

    calibrated = np.copy(measurements)
    for m in calibrated:
        m[:] = C @ (m - b)
    return calibrated


# === STANDSTILL DETECTION ===
def generate_standstill_flags(imu_data):
    """
    Generate binary standstill flags based on gyroscope activity.
    1 = standstill, 0 = motion.
    """
    standstill = np.zeros(len(imu_data))
    counter_after_motion = 60
    cooldown = 60

    for i, m in enumerate(imu_data):
        if np.linalg.norm(m[3:6]) < 0.13:  # gyroscope quiet
            counter_after_motion += 1
            if counter_after_motion > cooldown:
                standstill[i] = 1
        else:
            standstill[i] = 0
            standstill[i - cooldown:i] = 0
            counter_after_motion = 0

    return standstill


# === PLOTTING ===
def plot_accelerations_before_and_after(accs, accs_calibrated):
    """
    Plot acceleration magnitude before and after calibration.
    """
    times = np.arange(0, len(accs_calibrated)) * 0.01

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(times, np.linalg.norm(accs, axis=1), color='red', alpha=0.7, label="Uncalibrated norm")
    ax.plot(times, np.linalg.norm(accs_calibrated, axis=1), color='green', alpha=0.7,
            label=f"Calibrated norm $g_{{mean}}$ = {np.linalg.norm(accs_calibrated, axis=1).mean():.2f} m/s²")
    ax.axhline(9.81, color='blue', linestyle='--', alpha=0.5, label="Ideal norm $g_{ideal}$ = 9.81 m/s²")

    ax.grid(True)
    ax.set(
        xlabel="Time [s]",
        ylabel="Acceleration [m/s²]",
        ylim=[9.81 - 1.5, 9.81 + 1.5],
        title="Accelerometer Calibration Results"
    )
    ax.legend()
    plt.tight_layout()
    #plt.savefig(FIG, dpi=500)
    plt.show()


# === MAIN EXECUTION ===
if __name__ == "__main__":
    standstill_flags = generate_standstill_flags(imu_data)
    accs = accs[standstill_flags > 0]
    accs_calibrated = calibrate_accelerometer(accs, acc_params)
    plot_accelerations_before_and_after(accs, accs_calibrated)
