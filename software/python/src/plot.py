import serial, time
import numpy as np
import matplotlib.pyplot as plt

# === Konfiguration ===
PORT = "/dev/ttyUSB0"
BAUD = 115200

# === Kalibrierparameter (aktualisiert) ===
acc_params = np.array([0.0044, 0.0038, 0.0212,
                       0.0017, 0.0001, 0.0065,
                       1.0825, -0.0748, 0.2896])

gyr_params = np.array([-0.0041, -0.0071, 0.0120,
                       -0.0727, 0.0065, -0.0117,
                       -0.0273, 0.0177, -0.0335,
                       -0.0013, 0.0168, 0.0057])

# === Hilfsfunktionen ===
def upper_tri_matrix(sx, sy, sz, noxy, noxz, noyz):
    return np.array([[sx,  noxy, noxz],
                     [0.0, sy,   noyz],
                     [0.0, 0.0,  sz]])

def skew(v):
    x, y, z = v
    return np.array([[0, -z,  y],
                     [z,  0, -x],
                     [-y, x,  0]])

# === ACC ===
Sx_a, Sy_a, Sz_a, NOx_a, NOy_a, NOz_a, Bx_a, By_a, Bz_a = acc_params
b_a = np.array([Bx_a, By_a, Bz_a])
M_a = upper_tri_matrix(Sx_a, Sy_a, Sz_a, NOx_a, NOy_a, NOz_a)

# === GYR ===
Sx_g, Sy_g, Sz_g, NOx_g, NOy_g, NOz_g, Bx_g, By_g, Bz_g, Ex, Ey, Ez = gyr_params
b_g = np.array([Bx_g, By_g, Bz_g])
M_g = upper_tri_matrix(Sx_g, Sy_g, Sz_g, NOx_g, NOy_g, NOz_g)
eps = np.array([Ex, Ey, Ez])  # evtl. np.deg2rad(), falls in Grad
R = np.eye(3) + skew(eps)

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

window = 50
acc_raw_buf = np.zeros((window, 3))
gyr_raw_buf = np.zeros((window, 3))
acc_cal_buf = np.zeros((window, 3))
gyr_cal_buf = np.zeros((window, 3))

# === Verbindung ===
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
            # Wenn z. B. "-0.02710.28302" oder Nullbytes auftauchen
            print(f"⚠️  Ungültige Zeile übersprungen: {line!r}")
            continue
        acc = np.array([ax, ay, az])
        gyr = np.array([gx, gy, gz])

        # === Kalibrierung anwenden ===
        acc_cal = np.linalg.inv(np.eye(3)+M_a) @ (acc - b_a)
        gyr_cal = np.linalg.inv(np.eye(3)+M_g) @ (np.linalg.inv(R) @ (gyr - b_g))

        # === In Buffers schieben ===
        acc_raw_buf = np.vstack((acc_raw_buf[1:], acc))
        gyr_raw_buf = np.vstack((gyr_raw_buf[1:], gyr))
        acc_cal_buf = np.vstack((acc_cal_buf[1:], acc_cal))
        gyr_cal_buf = np.vstack((gyr_cal_buf[1:], gyr_cal))

        # === Plot updaten ===
        for i in range(3):
            lines[i][0].set_data(np.arange(window), acc_raw_buf[:, i])
            lines[i][1].set_data(np.arange(window), acc_cal_buf[:, i])
            axs[0, i].set_xlim(0, window)
            axs[0, i].relim(); axs[0, i].autoscale_view()

            lines[i+3][0].set_data(np.arange(window), gyr_raw_buf[:, i])
            lines[i+3][1].set_data(np.arange(window), gyr_cal_buf[:, i])
            axs[1, i].set_xlim(0, window)
            axs[1, i].relim(); axs[1, i].autoscale_view()

        plt.pause(0.001)

except KeyboardInterrupt:
    print("\n🛑 Stop Raw Read gesendet.")
    ser.write(b"Stop Raw Read\n")
    ser.close()
    plt.ioff()
    plt.show()
