#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the GUI for the motion analysis.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

from os import error
from PyQt5.QtWidgets import QComboBox, QGridLayout, QLabel, QMainWindow, QPushButton, QSizePolicy, QWidget, QVBoxLayout
from pyqtgraph.Qt import QtGui, QtCore
import sys
from PyQt5.QtCore import QObject, QThread, pyqtSignal
import numpy as np
import pyqtgraph as pg
from pyqtgraph.graphicsItems.CurvePoint import CurveArrow
from pyqtgraph.graphicsItems.PlotCurveItem import PlotCurveItem
from pyqtgraph.graphicsItems.PlotItem.PlotItem import PlotItem
from pyqtgraph.widgets.ProgressDialog import ProgressDialog


# UPDATE_RATE = 30

RED = (217, 83, 25)

import rospy as rp


class FeatureGraphsWidget(QWidget):
    update = pyqtSignal(np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray)
    def __init__(self):
        super().__init__()
        self.update.connect(self._update)

        self.setLayout(QGridLayout())
        layout = self.layout()

        self.user_trajectory = pg.PlotWidget(title="user_trajectory")
        self.reference_trajectory = pg.PlotWidget(title="reference_trajectory")
        self.discrete_user_trajectory = pg.PlotWidget(title="discrete_user_trajectory")
        self.discrete_reference_trajectory = pg.PlotWidget(title="discrete_reference_trajectory")
        self.errors = pg.PlotWidget(title="errors")
        self.progress_vector = pg.PlotWidget(title="progress_vector")
        self.progress_vector.setAspectLocked()
        self.progress_vector.setXRange(-1, 1)
        self.progress_vector.setYRange(-1, 1)
        self.feature_progression = pg.PlotWidget(title="feature_progression")

        self.user_trajectory_curve = pg.PlotCurveItem()
        self.reference_trajectory_curve = pg.PlotCurveItem()
        self.discrete_user_trajectory_curve = pg.PlotCurveItem()
        self.discrete_reference_trajectory_curve = pg.PlotCurveItem()
        self.errors_curve = pg.PlotCurveItem()
        self.progress_vector_curve = pg.PlotCurveItem()

 
        self.user_trajectory.addItem(self.user_trajectory_curve)
        self.reference_trajectory.addItem(self.reference_trajectory_curve)
        self.discrete_user_trajectory.addItem(self.discrete_user_trajectory_curve)
        self.discrete_reference_trajectory.addItem(self.discrete_reference_trajectory_curve)
        self.errors.addItem(self.errors_curve)
        self.progress_vector.addItem(self.progress_vector_curve)

        self.feature_index_pointer = CurveArrow(self.discrete_reference_trajectory_curve, 0)
        self.feature_index_pointer.setStyle(angle=90, headWidth=5, pen=RED, brush= RED)

        layout.addWidget(self.progress_vector, 0, 0, 1, 1)
        layout.addWidget(self.user_trajectory, 1, 0, 4, 1)
        layout.addWidget(self.reference_trajectory, 5, 0, 4, 1)
        layout.addWidget(self.discrete_user_trajectory, 9, 0, 4, 1)
        layout.addWidget(self.discrete_reference_trajectory, 13, 0, 4, 1)
        layout.addWidget(self.errors, 17, 0, 4, 1)

        self.frozen = False

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, w):
        return w * 3
    
    @QtCore.pyqtSlot(np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray)
    def _update(self, user_trajectory, reference_trajectory, discrete_user_trajectory, discrete_reference_trajectory, errors, progress_vector, prediction):
        if self.frozen:
            return
        
        user_trajectory_x = np.array(range(len(user_trajectory)))
        user_trajectory_y = user_trajectory
        self.user_trajectory_curve.setData(user_trajectory_x, user_trajectory_y)
        
        discrete_reference_trajectory_x = np.array(range(len(reference_trajectory)))
        discrete_reference_trajectory_y = reference_trajectory 
        self.reference_trajectory_curve.setData(discrete_reference_trajectory_x, discrete_reference_trajectory_y)
        
        discrete_user_trajectory_x = np.array(range(len(discrete_user_trajectory)))
        discrete_user_trajectory_y = discrete_user_trajectory
        self.discrete_user_trajectory_curve.setData(discrete_user_trajectory_x, discrete_user_trajectory_y)
        
        discrete_reference_trajectory_x = np.array(range(len(discrete_reference_trajectory)))
        discrete_reference_trajectory_y = discrete_reference_trajectory
        
        self.discrete_reference_trajectory_curve.setData(discrete_reference_trajectory_x, discrete_reference_trajectory_y)
        self.feature_index_pointer.setIndex(prediction)
        
        errors_x = np.array(range(len(errors)))
        errors_y = errors
        
        self.errors_curve.setData(errors_x, errors_y)

        # TODO: draw vector here? https://stackoverflow.com/questions/44246283/how-to-add-a-arrow-head-to-my-line-in-pyqt4
        self.progress_vector_curve.setData(progress_vector, pen = RED)


class MotionAnaysisGUI(QMainWindow):
    update_signal = pyqtSignal()
    def __init__(self):
        self.app = QtGui.QApplication([])
        super().__init__()
        self.update_signal.connect(self.update)

        sizePolicy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy.setHeightForWidth(True)
        self.setSizePolicy(sizePolicy)

        self.controls = QWidget(self)
        self.controls.setLayout(QVBoxLayout())
        controls_layout = self.controls.layout()

        self.overall_progress_vector_widget = pg.PlotWidget(title="overall_progress_vector")
        self.overall_progress_vector_widget.setAspectLocked()
        self.overall_errors_widget = pg.PlotWidget(title="overall_errors")
        controls_layout.addWidget(self.overall_progress_vector_widget)
        controls_layout.addWidget(self.overall_errors_widget)

        label1 = QLabel(self.controls)
        label1.setText("Spot to display:")

        self.spot_chooser = QComboBox(self)
        self.spot_chooser.setEditable(False)
        self.spot_chooser.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        self.spot_chooser.currentTextChanged.connect(self.choose_spot)

        controls_layout.addWidget(label1)
        controls_layout.addWidget(self.spot_chooser, 1)
        btn1 = QPushButton("freeze graphs")
        btn1.clicked.connect(self.trigger_freeze_graphs)
        controls_layout.addWidget(btn1, 1)
        
        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=False)

        self.setWindowTitle("Motion Analysis GUI")

        self.feature_widgets = dict()

        self.spot_data = dict()

        self.chosen_spot = None
        self.create_graphs()

        self.show()
    
    def heightForWidth(self, width):
        return width * (1 + (len(self.spot_data.get(self.chosen_spot).get("feature_hashes") or 0)))

    def update_available_spots(self, spot_name, active, feature_hashes=[]):
        """When a spot goes active and has an exercise, update the available spots for our drop down menu."""
        if active:
            # if self.spot_chooser.findData(str(spot_name)): # If spot was already added
            #     return
            self.spot_chooser.addItem(str(spot_name))
            self.spot_data[str(spot_name)] = {"feature_hashes": feature_hashes}
        else:
            idx = self.spot_chooser.findData(str(spot_name))
            self.spot_chooser.removeItem(idx)
            try:
                del self.spot_data[spot_name]
            except KeyError: # For now, if whe start the GUI at a "bad" time, ignore this step because the spot what never active
                pass

        self.update()

    @QtCore.pyqtSlot(str)
    def choose_spot(self, spot):
        self.chosen_spot = spot
        self.create_graphs()

    @QtCore.pyqtSlot()
    def trigger_freeze_graphs(self):
        for fw in self.feature_widgets.values():
            fw.frozen = not fw.frozen

    def create_graphs(self):
        """Our main window has a container with all the graphs for all features of an exercise. """
        container = QWidget()
        container.setLayout(QGridLayout())

        spot_data = self.spot_data.get(self.chosen_spot)

        layout = container.layout()

        # label1 = QLabel(self.controls)
        # label1.setText("Aggregated data:")
        # layout.addWidget(label1, 0, 0, 4, 1)

        layout.addWidget(self.controls, 1, 1, 4, 1)

        # Feature specific data

        if spot_data:
            feature_hashes = spot_data["feature_hashes"]
            for i, h in enumerate(feature_hashes):
                feature_widget = FeatureGraphsWidget()
                self.feature_widgets[h] = feature_widget
                layout.addWidget(feature_widget, 1, 5 + i, 10, 1)

        #     self.setFixedWidth(300 + (len(feature_hashes) * 300))
        # else:
        #     pass
        #     self.setFixedWidth(300)
        
        self.setFixedHeight(900)
    
        self.setCentralWidget(container)

        self.update()
        

    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()


class GUIHandler(QThread):
    

    def __init__(self):
        super().__init__()

    def run(self, gui):
        gui.start()