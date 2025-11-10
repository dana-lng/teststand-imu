from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QVBoxLayout, QWidget
import os
import pandas as pd
import sys

# src/gui/plot_gui/plot_utils.py
import time, serial
from collections import deque
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QVBoxLayout, QWidget
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas


class LivePlot:
    def __init__(self, target_widget, ser):
        self.ser = ser  # <- Serielle Verbindung kommt von außen
 
        # --- Layout vorbereiten ---
        self.canvas = FigureCanvas(Figure(figsize=(5, 3)))
        self.ax = self.canvas.figure.add_subplot(111)
        # Wenn das Zielwidget kein Layout hat, erstellen
        layout = target_widget.layout()
        if layout is None:
            layout = QVBoxLayout(target_widget)
            target_widget.setLayout(layout)
        layout.addWidget(self.canvas)

        # --- Plot vorbereiten ---
        self.ax.set_title("Live Sensor Data")
        self.ax.set_xlabel("Samples")
        self.ax.set_ylabel("ax")
        self.line, = self.ax.plot([], [], color="tab:blue", label="ax")
        self.ax.legend()
        self.ax.grid(True)

        # --- Datenpuffer ---
        self.max_points = 200
        self.xdata = deque(maxlen=self.max_points)
        self.ydata = deque(maxlen=self.max_points)
        self.index = 0
      
        # --- Timer ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)  # alle 50 ms

    def update_plot(self):
        line = self.ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            return
        if "Calibration Done" in line:
            print("Kalibrierung abgeschlossen")
            self.timer.stop()
            return

        parts = line.split(",")
        if len(parts) == 6:
            ax, ay, az, gx, gy, gz = map(float, parts)
            self.index += 1
            self.xdata.append(self.index)
            self.ydata.append(ax)

            self.line.set_data(self.xdata, self.ydata)
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw_idle()


    def stop(self):
        if self.timer.isActive():
            self.timer.stop()
        if self.ser.is_open:
            self.ser.close()
        self.index = 0
        self.xdata.clear()
        self.ydata.clear()
        self.canvas.draw_idle()


def apply_stylesheet(widget: QWidget, css_filename: str):
    # Absoluten Pfad zur CSS-Datei ermitteln (relativ zu dieser Datei)
    basePath = os.path.dirname(os.path.abspath(__file__))
    cssPath = os.path.join(basePath, css_filename)

    try:
        with open(cssPath, "r") as f:
            stylesheet = f.read()
            widget.setStyleSheet(stylesheet)
    except FileNotFoundError:
        print(f"Stylesheet {cssPath} wurde nicht gefunden.")


"""     
def setup_plot_1(target_widget, instance):

    fig = Figure(figsize=(3, 1.5))                              # Erstelle eine neue Matplotlib-Figur mit gegebener Größe
    canvas = FigureCanvas(fig)                                  # Der Canvas zeigt die Figur im Qt-Widget an
    ax = fig.add_subplot(111)                                   # Füge ein einzelnes Plot-Achsenobjekt hinzu
    #canvas.setFixedSize(600, 300)

    layout = QVBoxLayout()                                      # Erstelle ein vertikales Layout für das Ziel-Widget
    layout.setContentsMargins(0, 0, 0, 0)                       # Entferne Ränder (Padding)
    layout.addWidget(canvas)                                    # Füge den Canvas zum Layout hinzu

    
    if target_widget.layout() is not None:                       # Falls bereits ein Layout existiert, entferne es (wichtig für saubere Darstellung)
        QWidget().setLayout(target_widget.layout())

    
    target_widget.setLayout(layout)                             # Weise dem Widget das neue Layout zu

    
    line, = ax.plot([], [], color='black')                      # Erstelle eine leere Linie (für spätere Daten)


    # Achsenbeschriftungen und Titel
    ax.set_xlabel(r'Samples')                                          # x-Achse labeln
    ax.set_ylabel(r'ax')                                               # y-Achse labeln - Latex-Formel für die Länge ($...$)
    ax.set_title(r'Live Sensor Data')                                  # Titel oben setzen
    ax.legend() 
    ax.set_xlim(-100, 100)
    ax.set_ylim(-100, 100)                                                
    #fig.subplots_adjust(bottom=0.2)                             # 20% Abstand unten


    
    apply_stylesheet(target_widget, "stylesheet_plot.css")      # Wende das Stylesheet an (für z.B. Rahmen, Farben etc.)

    canvas.draw()
    
    return canvas, ax, line                                     # Rückgabe der wichtigen Objekte zur weiteren Verwendung im Code


def setup_plot_2(target_widget, instance):
    fig = Figure(figsize=(3, 1.5))                                # Erstelle eine neue Matplotlib-Figur mit gegebener Größe
    canvas = FigureCanvas(fig)                                  # Der Canvas zeigt die Figur im Qt-Widget an
    ax = fig.add_subplot(111)                                   # Füge ein einzelnes Plot-Achsenobjekt hinzu
    #canvas.setFixedSize(600, 300)

    layout = QVBoxLayout()                                      # Erstelle ein vertikales Layout für das Ziel-Widget
    layout.setContentsMargins(0, 0, 0, 0)                       # Entferne Ränder (Padding)
    layout.addWidget(canvas)                                    # Füge den Canvas zum Layout hinzu

    
    if target_widget.layout() is not None:                       # Falls bereits ein Layout existiert, entferne es (wichtig für saubere Darstellung)
        QWidget().setLayout(target_widget.layout())

    
    target_widget.setLayout(layout)                             # Weise dem Widget das neue Layout zu

    
    line, = ax.plot([], [], color='black')                      # Erstelle eine leere Linie (für spätere Daten)

    # Achsenbeschriftungen und Titel
    ax.set_xlabel(r'')                                          # x-Achse labeln
    ax.set_ylabel(r'   ')                                       # y-Achse labeln - Latex-Formel für die Länge ($...$)
    ax.set_title(r'')                                           # Titel oben setzen
    ax.legend() 
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)                                                # Zeige Legende an
    #fig.subplots_adjust(bottom=0.2)                             # 20% Abstand unten


    
    apply_stylesheet(target_widget, "stylesheet_plot.css")      # Wende das Stylesheet an (für z.B. Rahmen, Farben etc.)

    
    return canvas, ax, line                                     # Rückgabe der wichtigen Objekte zur weiteren Verwendung im Code
"""   