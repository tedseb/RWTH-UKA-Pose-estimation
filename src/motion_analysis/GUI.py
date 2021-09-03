#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the GUI for the motion analysis.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

from os import error
from PyQt5.QtWidgets import QApplication, QComboBox, QGridLayout, QLabel, QMainWindow, QPushButton, QSizePolicy, QWidget, QVBoxLayout
from pyqtgraph.Qt import QtGui, QtCore
import sys
from PyQt5.QtChart import QChart, QChartView, QLineSeries
from PyQt5.QtCore import QPointF
from PyQt5.QtGui import QPainter
from PyQt5.QtCore import Qt
import time
import numpy as np
from numpy import arange, sin, cos, pi
import pyqtgraph as pg
from pyqtgraph.graphicsItems.PlotItem.PlotItem import PlotItem
from pyqtgraph.widgets.ProgressDialog import ProgressDialog

dummy_trajectory_discrete = [1, 2, 3, 2, 1, 2, 1, 2, 3, 4, 5, 4, 3, 2, 1]
dummy_trajectory = [1, 1.3, 3, 2.1, 2.2, 2.1, 0.5, 2.3, 3, 4.5, 5.1, 5.3, 3, 2.1, 1]

class SpotGraphWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setLayout(QGridLayout())
        layout = self.layout()

        self.user_trajectory = pg.PlotWidget(title="user_trajectory")
        self.reference_trajectory = pg.PlotWidget(title="reference_trajectory")
        self.discrete_user_trajectory = pg.PlotWidget(title="discrete_user_trajectory")
        self.discrete_reference_trajectory = pg.PlotWidget(title="discrete_reference_trajectory")
        self.errors = pg.PlotWidget(title="errors")
        self.progress_vector = pg.PlotWidget(title="progress_vector")
        self.progress_vector.setAspectLocked()

        layout.addWidget(self.progress_vector, 0, 0, 1, 1)
        layout.addWidget(self.user_trajectory, 1, 0, 4, 1)
        layout.addWidget(self.reference_trajectory, 5, 0, 4, 1)
        layout.addWidget(self.discrete_user_trajectory, 9, 0, 4, 1)
        layout.addWidget(self.discrete_reference_trajectory, 13, 0, 4, 1)
        layout.addWidget(self.errors, 17, 0, 4, 1)

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, w):
        return w * 3
    
    def trace(self, name, data):
        user_trajectory_x = data["user_trajectory_t"]
        user_trajectory_y = data["user_trajectory_values"]

        self.user_trajectory.plot(user_trajectory_x, user_trajectory_y)

        reference_trajectory_x = data["reference_trajectory_t"]
        reference_trajectory_y = data["reference_trajectory_values"]

        self.reference_trajectory.plot(reference_trajectory_x, reference_trajectory_y)

        discrete_user_trajectory_x = range(len(data["discrete_user_trajectory"]))
        discrete_user_trajectory_y = data["discrete_user_trajectory"]

        self.discrete_user_trajectory.plot(discrete_user_trajectory_x, discrete_user_trajectory_y)
        
        discrete_reference_trajectory_x = range(len(data["discrete_reference_trajectory"]))
        discrete_reference_trajectory_y = data["discrete_reference_trajectory"]

        self.discrete_reference_trajectory.plot(discrete_reference_trajectory_x, discrete_reference_trajectory_y)

        errors_x = range(len(data["errors"]))
        errors_y = data["errors"]

        self.errors.plot(errors_x, errors_y)

        progress_vector = data["progress_vector"]

        # TODO: draw vector here
        self.progress_vector.plot(progress_vector)

        if name in self.graphs:
            self.graphs[name].setData(dataset_x, dataset_y)
        else:
            self.graphs[name]["plot"] = self.canvas.plot(pen='y')


class MotionAnaysisGUI(QMainWindow):
    def __init__(self):
        self.app = QtGui.QApplication([])
        super().__init__()
        self.setLayout(QVBoxLayout())
        sizePolicy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy.setHeightForWidth(True)
        self.setSizePolicy(sizePolicy)

        self.controls = QWidget(self)
        self.controls.setLayout(QVBoxLayout())
        controls_layout = self.controls.layout()

        label1 = QLabel(self.controls)
        label1.setText("Spot to display:")

        self.spot_chooser = QComboBox(self)
        self.spot_chooser.setEditable(False)
        self.spot_chooser.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)

        controls_layout.addWidget(label1)
        controls_layout.addWidget(self.spot_chooser, 1)
        btn1 = QPushButton("freeze graphs")
        controls_layout.addWidget(btn1, 1)
        
        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=False)

        self.setWindowTitle("Motion Analysis GUI")

        self.graphs = dict()

        self.chosen_spot = None
        self.spot_data = dict()

        self.chosen_spot = "Spot with 2 features"
        self.update_available_spots("Spot with 1 feature", True, 1)
        self.update_available_spots("Spot with 2 features", True, 2)
        self.create_graphs()

        self.show()

    
    def heightForWidth(self, width):
        return width * (1 + self.spot_data[self.chosen_spot]["num_features"])

    def update_available_spots(self, spot_name, active, num_features=0):
        """When a spot goes active and has an exercise, update the available spots for our drop down menu."""
        if active:
            self.spot_chooser.addItem(spot_name)
            self.spot_data[spot_name] = {"num_features": num_features}
        else:
            self.spot_chooser.removeItem(spot_name)
            del self.spot_data[spot_name]

    def create_graphs(self):
        """Our main window has a container with all the graphs for all features of an exercise. """
        container = QWidget()
        container.setLayout(QGridLayout())

        num_features = self.spot_data[self.chosen_spot]["num_features"]

        layout = container.layout()

        label1 = QLabel(self.controls)
        label1.setText("Aggregated data:")
        layout.addWidget(label1, 0, 0, 4, 1)

        overall_progress_vector = pg.PlotWidget(title="overall_progress_vector")
        overall_progress_vector.setAspectLocked()
        overall_errors = pg.PlotWidget(title="overall_errors")
        layout.addWidget(overall_progress_vector, 0, 0, 2, 1)
        layout.addWidget(overall_errors, 2, 0, 4, 1)

        for i in range(1, num_features + 1):
            spot_widget = SpotGraphWidget()
            self.graphs[i] = {"user_trajectory": spot_widget.user_trajectory, \
                "discrete_user_trajectory": spot_widget.discrete_user_trajectory, \
                "reference_trajectory": spot_widget.reference_trajectory, \
                    "errors": spot_widget.errors, \
                        "progress_vector": spot_widget.progress_vector}
            layout.addWidget(spot_widget, 0, i, 10, 1)

        layout.addWidget(self.controls, 6, 0, 4, 1)

        self.setFixedWidth(300 + num_features * 300)
        self.setFixedHeight(800)
 
        self.setCentralWidget(container)
        

    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()



# Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    gui = MotionAnaysisGUI()
    i = 0

    def update():
        global gui, i
        t = np.arange(0, 3.0, 0.01)
        s = t + i
        gui.trace("sin", t, s)
        i += 0.1

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)

    gui.start()


