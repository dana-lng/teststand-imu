"""Live IMU validation tool.

Reads raw IMU lines from a serial device. Applies stored calibration for accelerometer and
gyroscope, plots raw vs calibrated values in real time,

Configure PORT below.
"""
import time
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import PillowWriter
import os
import pandas as pd
import imageio

# === PATH CONFIGURATION ===
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(BASE_DIR, "../../..", "results")
DOCS_DIR = os.path.join(BASE_DIR, "../../..", "docs")
GIF = os.path.join(DOCS_DIR, "live_validation.gif")
ACCEL_CALIB = os.path.join(RESULTS_DIR, "accel_calib_params.csv")
GYRO_CALIB = os.path.join(RESULTS_DIR, "gyro_calib_params.csv")

# === Configuration ===
PORT = "/dev/ttyUSB0" # CONFIGURE THIS PORT!
BAUD = 115200
WINDOW = 50

# === Calibration parameters ===
acc_params = pd.read_csv(ACCEL_CALIB).to_numpy().flatten()
gyr_params = pd.read_csv(GYRO_CALIB).to_numpy().flatten()


# === Helper functions ===
def upper_tri_matrix(sx, sy, sz, noxy, noxz, noyz):
    """Creates the upper triangular matrix (scale and non-orthogonality errors)."""
    return np.array([
        [sx,  noxy, noxz],
        [0.0, sy,   noyz],
        [0.0, 0.0,  sz]
    ])


def skew(v):
    """Creates the skew-symmetric matrix of a 3D vector (for cross product)."""
    x, y, z = v
    return np.array([
        [0, -z,  y],
        [z,  0, -x],
        [-y, x,  0]
    ])


# === Accelerometer ===
Sx_a, Sy_a, Sz_a, NOx_a, NOy_a, NOz_a, Bx_a, By_a, Bz_a = acc_params
b_a = np.array([Bx_a, By_a, Bz_a])
M_a = upper_tri_matrix(Sx_a, Sy_a, Sz_a, NOx_a, NOy_a, NOz_a)

# === Gyroscope ===
Sx_g, Sy_g, Sz_g, NOx_g, NOy_g, NOz_g, Bx_g, By_g, Bz_g, Ex, Ey, Ez = gyr_params
b_g = np.array([Bx_g, By_g, Bz_g])
M_g = upper_tri_matrix(Sx_g, Sy_g, Sz_g, NOx_g, NOy_g, NOz_g)
eps = np.array([Ex, Ey, Ez])
R = np.eye(3) + skew(eps)

# === GIF recording setup ===
save_gif = False
gif_duration_sec = 5
gif_fps = 30
gif_start_delay = 3

frames = []
frame_interval = 1 / gif_fps
start_time = None

# === Matplotlib setup ===
plt.ion()
fig, axs = plt.subplots(2, 3, figsize=(12, 6))
labels = ['AX', 'AY', 'AZ', 'GX', 'GY', 'GZ']
lines = []

for i, ax in enumerate(axs.flat):
    ax.set_title(labels[i])
    ax.grid(True)
    line_raw, = ax.plot([], [], lw=1, label='raw')
    line_cal, = ax.plot([], [], lw=1.5, label='cal')
    lines.append((line_raw, line_cal))

axs[0, 1].legend(loc="upper right")

acc_raw_buf = np.zeros((WINDOW, 3))
gyr_raw_buf = np.zeros((WINDOW, 3))
acc_cal_buf = np.zeros((WINDOW, 3))
gyr_cal_buf = np.zeros((WINDOW, 3))


# === Serial connection ===
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)
ser.write(b"Start Raw Read\n")
print("📡 Start Raw Read gesendet. Lese IMU-Daten... (Strg+C zum Abbrechen)")

try:
    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line or "Calibration" in line:
            continue

        parts = line.split(",")
        if len(parts) != 6:
            continue

        try:
            ax, ay, az, gx, gy, gz = map(float, parts)
        except ValueError:
            print(f"⚠️  Ungültige Zeile übersprungen: {line!r}")
            continue

        acc = np.array([ax, ay, az])
        gyr = np.array([gx, gy, gz])

        # === Apply calibration ===
        acc_cal = np.linalg.inv(np.eye(3) + M_a) @ (acc - b_a)
        gyr_cal = np.linalg.inv(np.eye(3) + M_g) @ (np.linalg.inv(R) @ (gyr - b_g))

        # === Update data buffers ===
        acc_raw_buf = np.vstack((acc_raw_buf[1:], acc))
        gyr_raw_buf = np.vstack((gyr_raw_buf[1:], gyr))
        acc_cal_buf = np.vstack((acc_cal_buf[1:], acc_cal))
        gyr_cal_buf = np.vstack((gyr_cal_buf[1:], gyr_cal))

        # === Update plot ===
        for i in range(3):
            # Accelerometer
            lines[i][0].set_data(np.arange(WINDOW), acc_raw_buf[:, i])
            lines[i][1].set_data(np.arange(WINDOW), acc_cal_buf[:, i])
            axs[0, i].set_xlim(0, WINDOW)
            axs[0, i].relim()
            axs[0, i].autoscale_view()

            # Gyroscope
            lines[i + 3][0].set_data(np.arange(WINDOW), gyr_raw_buf[:, i])
            lines[i + 3][1].set_data(np.arange(WINDOW), gyr_cal_buf[:, i])
            axs[1, i].set_xlim(0, WINDOW)
            axs[1, i].relim()
            axs[1, i].autoscale_view()

        plt.pause(0.001)

        # === Record frames for GIF ===
        if save_gif:
            if start_time is None:
                start_time = time.time()

            elapsed = time.time() - start_time
            if elapsed < gif_start_delay:
                continue  # skip capturing until delay time passes

            frame = np.frombuffer(fig.canvas.tostring_argb(), dtype='uint8')
            frame = frame.reshape(fig.canvas.get_width_height()[::-1] + (4,))
            frame = frame[..., 1:]
            frames.append(frame.copy())

            if elapsed > gif_start_delay + gif_duration_sec:
                save_gif = False


except KeyboardInterrupt:
    if frames:
        print("💾 Saving 5-second GIF...")
        ani = animation.ArtistAnimation(fig, [], interval=frame_interval * 1000)
        writer = PillowWriter(fps=gif_fps)
        imageio.mimsave(GIF, frames, fps=gif_fps)
        print(f"✅ GIF saved to {GIF}")
    print("\n🛑 Stop Raw Read gesendet.")
    ser.write(b"Stop Raw Read\n")
    ser.close()
    plt.ioff()
    plt.show()
