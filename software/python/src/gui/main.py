from PyQt5.QtWidgets import QMainWindow, QApplication, QMenu, QAction
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

# hinzufügen des Stylesheets
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from style.stylesheet import shadow_right_down, shadow_left_down, shadow_right_up, shadow_left_up

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from plot_gui.plot_utils import setup_plot

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        # Pfad zur UI ermitteln
        scriptDir = os.path.dirname(os.path.abspath(__file__))
        uiPath = os.path.join(scriptDir, "untitled.ui")

        # UI laden
        uic.loadUi(uiPath, self)

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
        shadow_left_down(self.plot_frame)
        shadow_right_down(self.textausgabe)
        shadow_right_down(self.pushButton_1)
        shadow_right_down(self.pushButton_2)
        shadow_right_down(self.pushButton_3)
        shadow_left_down(self.saveButton)
        shadow_left_down(self.exitButton)

        # Funktionalitäten der Buttons
        #self.pushButton_1.clicked.connect()
        #self.pushButton_2.clicked.connect()
        #self.pushButton_3.clicked.connect()

        #self.saveButton.clicked.connect(self.save)
        self.exitButton.clicked.connect(self.close)

        # Plot plotten
        self.canvasPlot, self.axPlot, self.Plot = setup_plot(self.plot_frame, self)
    
    def save(self):
        daten = []

        # CSV-Datei speichern
        with open("daten.csv", "w", newline="", encoding="utf-8") as datei:
            writer = csv.writer(datei, delimiter=";")
            writer.writerows(daten)

        if os.path.getsize(daten.csv) > 0:
            print("Daten wurden gespeichert!")
        else:
            print("Daten wurden nicht gespeichert!")

   # def calibration(self):




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