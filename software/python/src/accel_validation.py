import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd

# Pfade
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE_DIR, "../../..", "data")
DOCS_DIR = os.path.join(BASE_DIR, "../../..", "docs")
CSV = os.path.join(DATA_DIR, "data.csv")
FIG = os.path.join(DOCS_DIR, "acc_calibration_results.png")

imu_data = pd.read_csv(CSV).to_numpy()
accs = imu_data[:,0:3]

# Beispiel-Kalibrierungsparameter für den Beschleunigungssensor (Anpassung erforderlich)
acc_params = np.array([0.0044, 0.0038, 0.0212,
                       0.0017, 0.0001, 0.0065,
                       1.0825, -0.0748, 0.2896])

def calibrate_accelerometer(measurements, sensor_errors):
    '''
    Calibrate sensor measurements according to the model.
    See eq. (7).
    '''
    M, b = sensor_error_model(sensor_errors[0:9])
    C = np.linalg.inv(M)

    calibrated_measurements = np.copy(measurements)
    for measurement in calibrated_measurements:
        measurement[:] = C @ (measurement - b)

    return calibrated_measurements

def plot_accelerations_before_and_after(accs, accs_calibrated):
    fig, ax = plt.subplots(1, 1, figsize=(8, 4))
    times = np.arange(0, len(accs_calibrated)) * 0.01
    ax.plot(times, np.linalg.norm(accs, axis=1), alpha = 0.7, color='red')
    ax.plot(times, np.linalg.norm(accs_calibrated, axis=1), alpha = 0.7, color='green')
    ax.axhline(y=9.81, color='blue', linestyle='--', alpha=0.5)
    ax.grid()
    ax.legend(["Uncalibrated norm", f"Calibrated norm $g_{{mean}}$ = {np.linalg.norm(accs_calibrated, axis=1).mean():.2f} m/s²", "Ideal norm $g_{{ideal}}$ = 9.81 m/s²"])
    ax.set(xlabel='Time $[s]$', ylabel='Acceleration $[m/s^2]$', ylim = [9.81 - 1.5, 9.81 + 1.5])
    ax.set_title("Accelerometer Calibration Results")
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
accs = accs[standstill_flags > 0]
accs_calibrated = calibrate_accelerometer(accs, acc_params)
plot_accelerations_before_and_after(accs, accs_calibrated)