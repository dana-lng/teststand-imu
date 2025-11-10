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
    def __init__(self, target_widget, ser, indices=(0, 1, 2), labels=("ax", "ay", "az"), title="Accelerometer", ylabel="Acceleration [m/s²]", color=("tab:blue", "tab:orange", "tab:green")):
        """
        indices: welche Spalten aus der IMU-Zeile geplottet werden (z. B. (0,1,2) = ax, ay, az)
        labels:  Legendenbeschriftung
        ylabel:  Achsenlabel
        """
        self.ser = ser
        self.indices = indices
        self.labels = labels



        # --- Layout vorbereiten ---
        self.canvas = FigureCanvas(Figure(figsize=(5, 3)))
        self.ax = self.canvas.figure.add_subplot(111)
        layout = target_widget.layout()
        if layout is None:
            layout = QVBoxLayout(target_widget)
            target_widget.setLayout(layout)
        layout.addWidget(self.canvas)

        # --- Plot vorbereiten ---
        self.ax.set_title(title, fontsize=20)
        self.ax.set_ylabel(ylabel)
        self.lines = [
            self.ax.plot([], [], color=color[i], label=labels[i])[0]
            for i in range(len(labels))
        ]
        self.ax.legend()
        self.ax.grid(True)

        # --- Datenpuffer ---
        self.max_points = 20
        self.xdata = deque(maxlen=self.max_points)
        self.ydata = [deque(maxlen=self.max_points) for _ in range(len(labels))]
        self.index = 0

        # --- Timer ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)

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
            try:
                vals = list(map(float, parts))
            except ValueError:
                return

            self.index += 1
            self.xdata.append(self.index)
            for i, idx in enumerate(self.indices):
                self.ydata[i].append(vals[idx])
                self.lines[i].set_data(self.xdata, self.ydata[i])
            self.ax.xaxis.set_visible(False)
            self.ax.set_xlim(self.index - self.max_points, self.index)
            self.ax.relim()
            self.ax.autoscale_view(scalex=False)
            self.canvas.draw_idle()

    def stop(self):
        if self.timer.isActive():
            self.timer.stop()
        self.index = 0
        self.xdata.clear()
        for buf in self.ydata:
            buf.clear()
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
