#!/usr/bin/env python3
"""
Widget para visualizar nube de puntos 3D en PyQt5
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D


class Cloud3DViewer(QWidget):
    """Widget para mostrar nube de puntos 3D"""

    REDRAW_INTERVAL_MS = 1000  # max 1 redraw per second
    MAX_DRAW_POINTS = 5000     # matplotlib 3D gets slow above this

    def __init__(self, parent=None):
        super().__init__(parent)
        self.points = np.array([]).reshape(0, 3)
        self._dirty = False
        self.init_ui()

        # Redraw on a timer instead of on every message
        self.redraw_timer = QTimer(self)
        self.redraw_timer.timeout.connect(self._redraw_if_dirty)
        self.redraw_timer.start(self.REDRAW_INTERVAL_MS)

    def init_ui(self):
        """Inicializa la interfaz"""
        layout = QVBoxLayout()
        self.setLayout(layout)

        # Crea figura matplotlib
        self.figure = Figure(figsize=(6, 6), dpi=100, facecolor='#1a1a1a')
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111, projection='3d')

        layout.addWidget(self.canvas)
        self.setMinimumSize(400, 400)

        # Dibuja nube inicial vacía
        self.draw_cloud()

    def update_cloud(self, points):
        """Guarda la nube; el dibujo lo hace el timer"""
        if points is None or len(points) == 0:
            return
        arr = np.asarray(points)
        if arr.dtype.names:  # structured array with x, y, z fields
            arr = np.column_stack((arr['x'], arr['y'], arr['z']))
        self.points = arr.astype(np.float32).reshape(-1, 3)
        self._dirty = True

    def _redraw_if_dirty(self):
        if self._dirty and self.isVisible():
            self._dirty = False
            self.draw_cloud()

    def _setup_axes(self):
        """Configura ejes una sola vez (evita ax.clear() en cada redibujo)"""
        self.ax.set_facecolor('#1a1a1a')
        self.ax.set_xlabel('X (m)', color='white')
        self.ax.set_ylabel('Y (m)', color='white')
        self.ax.set_zlabel('Z (m)', color='white')
        self.ax.tick_params(colors='white')
        self.ax.grid(True, alpha=0.3)
        self.scatter = self.ax.scatter([], [], [], c=[], cmap='viridis',
                                       s=1, alpha=0.6, vmin=0.0, vmax=1.0)

    def draw_cloud(self):
        """Dibuja la nube de puntos (actualiza el scatter existente)"""
        if not hasattr(self, 'scatter'):
            self._setup_axes()

        pts = self.points
        if len(pts) > self.MAX_DRAW_POINTS:
            idx = np.random.choice(len(pts), self.MAX_DRAW_POINTS, replace=False)
            pts = pts[idx]

        if len(pts) > 0:
            z_min, z_max = pts[:, 2].min(), pts[:, 2].max()
            z_norm = (pts[:, 2] - z_min) / (z_max - z_min + 1e-6)
            self.scatter._offsets3d = (pts[:, 0], pts[:, 1], pts[:, 2])
            self.scatter.set_array(z_norm)

            margin = 1.0
            self.ax.set_xlim(pts[:, 0].min() - margin, pts[:, 0].max() + margin)
            self.ax.set_ylim(pts[:, 1].min() - margin, pts[:, 1].max() + margin)
            self.ax.set_zlim(z_min - 0.5, z_max + 0.5)
        else:
            self.scatter._offsets3d = (np.empty(0), np.empty(0), np.empty(0))
            self.scatter.set_array(np.empty(0))

        self.canvas.draw_idle()

    def get_points_count(self):
        """Retorna número de puntos"""
        return len(self.points)

    def get_bounds(self):
        """Retorna límites de la nube"""
        if len(self.points) == 0:
            return None

        return {
            'x_min': float(self.points[:, 0].min()),
            'x_max': float(self.points[:, 0].max()),
            'y_min': float(self.points[:, 1].min()),
            'y_max': float(self.points[:, 1].max()),
            'z_min': float(self.points[:, 2].min()),
            'z_max': float(self.points[:, 2].max()),
        }

    def clear(self):
        """Limpia la nube"""
        self.points = np.array([]).reshape(0, 3)
        self.draw_cloud()
