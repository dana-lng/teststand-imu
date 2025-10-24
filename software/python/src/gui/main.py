from PyQt5.QtWidgets import QMainWindow, QApplication, QMenu, QAction, QTableWidgetItem
from PyQt5 import uic 
import os
import sys
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QShortcut
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QSizePolicy, QWidget
from PyQt5.QtGui import QKeySequence
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QLinearGradient, QBrush, QColor
from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import csv
import os, time, serial, pandas as pd

# hinzufügen des Stylesheets
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from style.stylesheet import shadow_right_down, shadow_left_down, shadow_right_up, shadow_left_up

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from plot_gui.plot_utils import setup_plot_1, setup_plot_2 

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        # Pfad zur UI ermitteln
        scriptDir = os.path.dirname(os.path.abspath(__file__))
        uiPath = os.path.join(scriptDir, "untitled.ui")

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
        self.pushButton_2.clicked.connect(self.read_raw_data)
        self.pushButton_3.clicked.connect(self.stop_reading)

        #self.saveButton.clicked.connect(self.save)
        self.exitButton.clicked.connect(self.close)

        # Plot plotten
        self.canvasPlot, self.axPlot, self.Plot = setup_plot_1(self.plot_frame_1, self)
        # Plot plotten
        self.canvasPlot, self.axPlot, self.Plot = setup_plot_2(self.plot_frame_2, self)


    def calibration(self):
            # Basis: Ordner, in dem das aktuelle Skript liegt
            BASE_DIR = os.path.dirname(os.path.abspath(__file__))
            DATA_DIR = os.path.join(BASE_DIR, "../../..", "data")  # ../data relativ zu src/
            CSV = os.path.join(DATA_DIR, "data.csv")

            PORT = "COM3"
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
            
            # TODO Daten in Datei speichern

    def read_raw_data(self):
            
            PORT = "COM3"
            BAUD = 115200

            ser = serial.Serial(PORT, BAUD, timeout=1)
            time.sleep(2)

            print("Fange an Daten zu sammeln...")
            ser.write(b"Start Raw Read\n")

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
                        self.textausgabe_1.setText(f"ax = {ax}, ay= {ay}, az= {az}, gx = {gx}, gy = {gy}, gz = {gz}")
                            
            except KeyboardInterrupt:
                print("Manuell gestoppt")

    def stop_reading(self):
        PORT = "COM3"
        BAUD = 115200

        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)
        ser.write(b"Stop Raw Read\n")
        print("Auslesen beendet!")


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