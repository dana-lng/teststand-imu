import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd

# Pfade
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE_DIR, "../../..", "data")
DOCS_DIR = os.path.join(BASE_DIR, "../../..", "docs")
CSV = os.path.join(DATA_DIR, "data.csv")
FIG = os.path.join(DOCS_DIR, "gyro_calibration_results.png")

imu_data = pd.read_csv(CSV).to_numpy()
gyros = imu_data[:,3:6]

# Beispiel-Kalibrierungsparameter für den Gyroskopsensor (Anpassung erforderlich)
gyr_params = np.array([-0.0012, -0.0099, -0.0041,
                       0.0142,  0.0020, -0.0470,
                       -0.0187, -0.0017, -0.0301,
                       0.0009,  0.0175,  0.0018])

def skew(v):
    '''Returns skew-symmetric matrix, which satisfies A^T = -A'''
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

def misalignment(epsilon):
    '''
    Build rotation matrix from gyroscope coordinate frame
    to accelerometer coordinate frame according to eq. (5)
    '''
    return np.eye(3) + skew(epsilon)

def calibrate_gyroscope(measurements, sensor_errors, epsilon):
    '''
    Calibrate sensor measurements according to the model.
    See eq. (8).
    '''
    M, b = sensor_error_model(sensor_errors[0:9])
    C = np.linalg.inv(M)
    Rm = np.linalg.inv(misalignment(epsilon))

    calibrated_measurements = np.copy(measurements)
    for measurement in calibrated_measurements:
        measurement[:] = C @ Rm @ measurement - C @ b

    return calibrated_measurements

def plot_accelerations_before_and_after(gyros, gyros_calibrated):
    times = np.arange(0, len(gyros_calibrated)) * 0.01

    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    labels = ['gx', 'gy', 'gz']

    for i in range(3):
        axs[i].plot(times, gyros[:, i], color='red', alpha=0.7, label=f'Uncalibrated ${labels[i]}_{{mean}}$ = {gyros[:, i].mean():.4f} rad/s')
        axs[i].plot(times, gyros_calibrated[:, i], color='green', alpha=0.7, label=f'Calibrated ${labels[i]}_{{mean}}$ = {gyros_calibrated[:, i].mean():.4f} rad/s')
        axs[i].axhline(0, color='blue', linestyle='--', linewidth= 2, alpha=0.5, label=f'Ideal $g_{{ideal}}$ = 0 rad/s')
        axs[i].grid()
        axs[i].set_ylabel(f'{labels[i]} [rad/s]')
        axs[i].legend(loc='upper right')

    # Manual y axis limits for better visualization
    axs[0].set_ylim([-0.03, 0.013])
    axs[1].set_ylim([-0.005, 0.01])
    axs[2].set_ylim([-0.03, 0.017])

    axs[2].set_xlabel('Time [s]')
    fig.suptitle("Gyroscope Standstill Calibration Results", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    plt.savefig(FIG, dpi=500)
    plt.show()

def sensor_error_model(errors):
    '''
    Build sensor error model matrix and bias according to
    eq. (1) and (2) in the paper.
    '''
    KX, KY, KZ, NOX, NOY, NOZ, BX, BY, BZ = errors
    M = np.array([[1.0 + KX, NOY, NOZ],
                  [0.0, 1.0 + KY, NOX],
                  [0.0, 0.0, 1.0 + KZ]])
    b = np.array([BX, BY, BZ])

    return M, b


# Stillstandserkennung
def generate_standstill_flags(imu_data):
    standstill = np.zeros(len(imu_data))
    counter_after_motion = 60
    number = 60
    for i, m in enumerate(imu_data):
        if np.linalg.norm(m[3:6]) < 0.13:  # Gyro ruhig
            counter_after_motion += 1
            if counter_after_motion > number:
                standstill[i] = 1
        else:
            standstill[i] = 0
            standstill[i-number:i] = 0
            counter_after_motion = 0
    return standstill

standstill_flags = generate_standstill_flags(imu_data)
gyros = gyros[standstill_flags > 0]
gyros_calibrated = calibrate_gyroscope(gyros, gyr_params, gyr_params[9:12])
plot_accelerations_before_and_after(gyros, gyros_calibrated)
