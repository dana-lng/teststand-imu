### Für Linux Qt5-Plugins setzen ###
# Ausblenden, wenn nicht unter Linux gearbeitet wird
import os, sys
"""
base = os.path.join(sys.prefix, "lib", "python3.12", "site-packages", "PyQt5", "Qt5")
os.environ["LD_LIBRARY_PATH"] = os.path.join(base, "lib") + ":" + os.environ.get("LD_LIBRARY_PATH", "")
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = os.path.join(base, "plugins", "platforms")
os.environ["QT_QPA_PLATFORM"] = "xcb"
"""

from PyQt5.QtWidgets import QMainWindow, QApplication, QMenu, QAction, QTableWidgetItem
from PyQt5 import uic
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QShortcut
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QSizePolicy, QWidget
from PyQt5.QtGui import QKeySequence
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPainter, QLinearGradient, QBrush, QColor
from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import csv
import os, subprocess, time, serial, pandas as pd
from collections import deque

# hinzufügen des Stylesheets
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from style.stylesheet import shadow_right_down, shadow_left_down, shadow_right_up, shadow_left_up

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from plot_gui.plot_utils import LivePlot

PORT = "/dev/ttyUSB0"
BAUD = 115200

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        # Pfad zur UI ermitteln
        scriptDir = os.path.dirname(os.path.abspath(__file__))
        uiPath = os.path.join(scriptDir, "main.ui")

        # UI laden
        uic.loadUi(uiPath, self)

        self.textausgabe_1.setAlignment(Qt.AlignCenter)

        #Logos laden
        scriptDir = os.path.dirname(os.path.abspath(__file__))
        tu_logo_path = os.path.join(scriptDir, "logos", "tudresden_logo_clean.png")
        pixmap1 = QPixmap(tu_logo_path)
        scaled = pixmap1.scaled(250, 120, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.tu_logo.setPixmap(scaled)
        self.tu_logo.setAlignment(Qt.AlignCenter)
        self.tu_logo.setScaledContents(False)

        scriptDir = os.path.dirname(os.path.abspath(__file__))
        tu_logo_path = os.path.join(scriptDir, "logos", "itm_logo_tinted.png")
        pixmap2 = QPixmap(tu_logo_path)
        scaled = pixmap2.scaled(250, 120, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.itm_logo.setPixmap(scaled)
        self.itm_logo.setAlignment(Qt.AlignCenter)
        self.itm_logo.setScaledContents(False)


        # Schatten
        shadow_left_down(self.plot_frame_1)
        shadow_left_down(self.plot_frame_2)
        shadow_right_down(self.textausgabe_1)
        shadow_right_down(self.pushButton_1)
        shadow_right_down(self.pushButton_2)
        shadow_right_down(self.pushButton_3)
        shadow_left_down(self.exitButton)

        # Funktionalitäten der Buttons
        self.pushButton_1.clicked.connect(self.calibration)
        self.pushButton_2.clicked.connect(self.start_reading)
        self.pushButton_3.clicked.connect(self.stop_reading)
        self.exitButton.clicked.connect(lambda: (self.stop_reading(), self.close()))


    def calibration(self):
            # Basis: Ordner, in dem das aktuelle Skript liegt
            BASE_DIR = os.path.dirname(os.path.abspath(__file__))
            DATA_DIR = os.path.join(BASE_DIR, "../../../..", "data")  # ../data relativ zu src/
            CSV = os.path.join(DATA_DIR, "data.csv")
            self.textausgabe_1.setText("Calibration Started and Collecting Data ...")
            QApplication.processEvents() # Update GUI

            """
            ser = serial.Serial(PORT, BAUD, timeout=1)
            time.sleep(2)

            print("Fange an Daten zu sammeln...")

            ser.write(b"Start Calibration\n")

            rows = []

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
            """
            # Start Calibration Script
            calibrate_file = os.path.join(os.path.dirname(__file__), "..", "imu-calib", "calibrate_real_imu.py")
            calibrate_file = os.path.abspath(calibrate_file)
            self.textausgabe_1.setText("Calculating Parameters ...")
            QApplication.processEvents() # Update GUI
            subprocess.run([sys.executable, calibrate_file, "--sampling_frequency=100"])
            self.textausgabe_1.setText("Calibration Done. Files are saved.")
            QApplication.processEvents() # Update GUI



    def start_reading(self):
        # Alte Canvas in beiden Frames entfernen
        for frame in (self.plot_frame_1, self.plot_frame_2):
            layout = frame.layout()
            if layout:
                while layout.count():
                    child = layout.takeAt(0)
                    if child.widget():
                        child.widget().setParent(None)
                        child.widget().deleteLater()

        # Serielle Verbindung öffnen
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=1)
            time.sleep(2)
            self.ser.write(b"Start Raw Read\n")
            print("✅ Verbindung geöffnet und Datenstream gestartet")
        except serial.SerialException as e:
            print("❌ Konnte COM-Port nicht öffnen:", e)
            return

        # --- LivePlot für Acc ---
        self.live_plot_acc = LivePlot(
            target_widget=self.plot_frame_1,
            ser=self.ser,
            indices=(0, 1, 2),
            labels=("ax", "ay", "az"),
            title = "Accelerometer",
            ylabel="Acceleration [m/s²]"
        )

        # --- LivePlot für Gyro ---
        self.live_plot_gyro = LivePlot(
            target_widget=self.plot_frame_2,
            ser=self.ser,
            indices=(3, 4, 5),
            labels=("gx", "gy", "gz"),
            ylabel="Angular velocity [rad/s]",
            title = "Gyroscope",
            color=("tab:red", "tab:purple", "tab:gray")
        )

        # Timer für Textanzeige
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_one_line)
        self.timer.start(10)


    def read_one_line(self):

        rows = []

        line = self.ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            return
        if "Calibration Done" in line:
            print("Kalibrierung beendet, speichere Daten...")

        parts = line.split(",")
        if len(parts) == 6:
            try:
                ax, ay, az, gx, gy, gz = map(float, parts)
            except ValueError:
                return
            rows.append([ax, ay, az, gx, gy, gz])
            #print(parts)
            self.textausgabe_1.setText(f"ax = {ax:.3f} m/s², ay = {ay:.3f} m/s², az = {az:.3f} m/s², gx = {gx:.3f} rad/s, gy = {gy:.3f} rad/s, gz = {gz:.3f} rad/s")



    def stop_reading(self):
        self.ser.write(b"Stop Raw Read\n")
        print("Auslesen beendet!")
        self.timer.stop()

        if hasattr(self, "live_plot_acc"):
            self.live_plot_acc.stop()
        if hasattr(self, "live_plot_gyro"):
            self.live_plot_gyro.stop()

        if self.ser.is_open:
            self.ser.close()

def load_stylesheet(app):
    scriptDir = os.path.dirname(os.path.abspath(__file__))  # Pfad zur aktuellen .py-Datei
    stylesheetPath = os.path.join(scriptDir, "style", "stylesheet.css")
    stylesheetPath = os.path.normpath(stylesheetPath)  # macht den Pfad plattformunabhängig

    if not os.path.exists(stylesheetPath):
        print(f"❌ Stylesheet nicht gefunden: {stylesheetPath}")
        return
    with open(stylesheetPath, "r") as f:
        css = f.read()
        app.setStyle("Fusion")  # damit QSS-Eigenschaften wie background-color auch wirken
        app.setStyleSheet(css)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    load_stylesheet(app)
    ui = MainWindow()
    ui.show()
    sys.exit(app.exec_())