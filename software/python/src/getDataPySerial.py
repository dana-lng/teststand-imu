import os, time, serial, pandas as pd

# Basis: Ordner, in dem das aktuelle Skript liegt
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE_DIR, "../../..", "data")  # ../data relativ zu src/
CSV = os.path.join(DATA_DIR, "data.csv")

PORT = "/dev/ttyUSB0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

print("Fange an Daten zu sammeln...")
ser.write(b"Start Calibration\n")

rows = []
t0 = time.time()

try:
    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        if "Calibration Done" in line:
            print("Kalibrierung beendet, speichere Daten...")
            break

        parts = line.split(",")
        if len(parts) == 6:
            ax, ay, az, gx, gy, gz = map(float, parts)
            rows.append([ax, ay, az, gx, gy, gz])
            print(parts)
except KeyboardInterrupt:
    print("Manuell gestoppt")

# Datei speichern
os.makedirs(DATA_DIR, exist_ok=True)
df = pd.DataFrame(rows, columns=["ax","ay","az","gx","gy","gz"])
df.to_csv(CSV, index=False)
print(f"{len(df)} Zeilen gespeichert in {CSV}")
