from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QVBoxLayout, QWidget
import os

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

        

def setup_plot_1(target_widget, instance):
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
