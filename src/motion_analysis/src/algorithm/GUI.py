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

try:
    from motion_analysis.src.algorithm.AlgoConfig import GUI_FPS
    from motion_analysis.src.algorithm.logging import log
except ImportError:
    from src.algorithm.AlgoConfig import GUI_FPS
    from src.algorithm.logging import log

RED = (217, 83, 25)

class FeatureGraphsWidget(QWidget):
    """ Implements a set a graphs that are displayed vertically on top of each other in our GUI to show us feature data."""
    update_user_data = pyqtSignal(np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray)
    update_reference_plots = pyqtSignal(np.ndarray, np.ndarray)
    def __init__(self):
        super().__init__()
        self.update_user_data.connect(self._update_user_data)
        self.update_reference_plots.connect(self._update_reference_plots)

        self.setLayout(QGridLayout())
        layout = self.layout()

        # Initialize trajectories so that we can plot "something"
        self._user_trajectory_x = np.array([0])
        self._user_trajectory_y = np.array([0])
        self._discrete_user_trajectory_x = np.array([0])
        self._discrete_user_trajectory_y = np.array([0])
        self._errors_x = np.array([0])
        self._errors_y = np.array([0])
        self._progress_vector_x = np.array([0, 0])
        self._progress_vector_y = np.array([0, 1])
        self._prediction = 0

        # We use one plot widget per trajectory
        self.user_trajectory = pg.PlotWidget(title="user_trajectory")
        self.reference_trajectory = pg.PlotWidget(title="reference_trajectory")
        self.discrete_user_trajectory = pg.PlotWidget(title="discrete_user_trajectory")
        self.discrete_reference_trajectory = pg.PlotWidget(title="discrete_reference_trajectory")
        self.errors = pg.PlotWidget(title="errors")
        self.progress_vector = pg.PlotWidget(title="progress_vector")
        self.progress_vector.setXRange(-1, 1)
        self.progress_vector.setYRange(-1, 1)
        self.progress_vector.setAspectLocked()
        self.feature_progression = pg.PlotWidget(title="feature_progression")

        # For each plot widget, we initialize a curve
        self.user_trajectory_curve = pg.PlotCurveItem([0, 0])
        self.reference_trajectory_curve = pg.PlotCurveItem([0, 0])
        self.discrete_user_trajectory_curve = pg.PlotCurveItem([0, 0])
        self.discrete_reference_trajectory_curve = pg.PlotCurveItem([0, 0])
        self.errors_curve = pg.PlotCurveItem([0, 0])
        self.progress_vector_curve = pg.PlotCurveItem([0, 0])

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

        # Blocks all updates of plots
        self.frozen = False

        self.thread = QThread()

        # This timer saves us resources, because we do not update plots lots, but rather with GUI_FPS fps
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(1000 / GUI_FPS) # 10 FPS

        # Since we do not always re-plot reference plot data, we need to keep track of wether it has already been set
        self.reference_plot_data_set = False

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, w):
        return w * 3

    @QtCore.pyqtSlot(np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray)
    def _update_user_data(self, user_trajectory, discrete_user_trajectory, errors, progress_vector, prediction):
        """ Set for our user trajectory, progress and errors arrays, but do not call setData already."""
        if self.frozen:
            return

        self._user_trajectory_x = np.array(range(len(user_trajectory)))
        self._user_trajectory_y = user_trajectory
        
        self._discrete_user_trajectory_x = np.array(range(len(discrete_user_trajectory)))
        self._discrete_user_trajectory_y = discrete_user_trajectory
        
        self._errors_x = np.array(range(len(errors)))
        self._errors_y = errors
        
        # TODO: draw vector here? https://stackoverflow.com/questions/44246283/how-to-add-a-arrow-head-to-my-line-in-pyqt4
        self._progress_vector_x = np.array([0, progress_vector[0]])
        self._progress_vector_y = np.array([0, progress_vector[1]])

        self._prediction = prediction
    
    def update_plots(self):
        """ Call setData for all our user data, progress and error plots."""
        if self.frozen:
            return

        try:
            self.user_trajectory_curve.setData(self._user_trajectory_x, self._user_trajectory_y)
            self.discrete_user_trajectory_curve.setData(self._discrete_user_trajectory_x, self._discrete_user_trajectory_y)
            self.feature_index_pointer.setIndex(self._prediction)
            self.errors_curve.setData(self._errors_x, self._errors_y)
            # TODO: draw vector here? https://stackoverflow.com/questions/44246283/how-to-add-a-arrow-head-to-my-line-in-pyqt4
            self.progress_vector_curve.setData(self._progress_vector_x, self._progress_vector_y, pen = RED)
        except IndexError as e: # This occurs if our progress points to an index higher than our trajectory length
            log(e)

    @QtCore.pyqtSlot(np.ndarray, np.ndarray)
    def _update_reference_plots(self, reference_trajectory, discrete_reference_trajectory):
        """ Call setData for our reference trajectory plots."""
        if self.frozen:
            return

        reference_trajectory_x = np.array(range(len(reference_trajectory)))
        reference_trajectory_y = reference_trajectory 
        self.reference_trajectory_curve.setData(reference_trajectory_x, reference_trajectory_y)

        discrete_reference_trajectory_x = np.array(range(len(discrete_reference_trajectory)))
        discrete_reference_trajectory_y = discrete_reference_trajectory
        self.discrete_reference_trajectory_curve.setData(discrete_reference_trajectory_x, discrete_reference_trajectory_y)

        self.reference_plot_data_set = True


class MotionAnaysisGUI(QMainWindow):
    update_signal = pyqtSignal()
    update_overall_data_signal = pyqtSignal(int, int, np.ndarray)
    def __init__(self):
        self.app = QtGui.QApplication([])
        super().__init__()
        self.update_signal.connect(self.update)
        self.update_overall_data_signal.connect(self._update_overall_data)

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

        self.overall_progress_vector_widget.setXRange(-1, 1)
        self.overall_progress_vector_widget.setYRange(-1, 1)
        self.overall_progress_vector_widget.setAspectLocked()

        self.overall_progress_vector_curve = pg.PlotCurveItem([0, 0])
        self.overall_errors_curve = pg.PlotCurveItem([0, 0])

        self.overall_progress_vector_widget.addItem(self.overall_progress_vector_curve)
        self.overall_errors_widget.addItem(self.overall_errors_curve)

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
        self.create_feature_widgets()

        self.show()

    @QtCore.pyqtSlot(int, int, np.ndarray)
    def _update_overall_data(self, progress, alignment, progress_alignment_vector):
        self._progress_vector_x = np.array([0, progress_alignment_vector[0]])
        self._progress_vector_y = np.array([0, progress_alignment_vector[1]])
        self.overall_progress_vector_curve.setData(self._progress_vector_x, self._progress_vector_y, pen = RED)

    
    def heightForWidth(self, width):
        return width * (1 + (len(self.spot_data.get(self.chosen_spot).get("feature_hashes") or 0)))

    def update_available_spots(self, spot_name, active, feature_hashes=[]):
        """When a spot goes active and has an exercise, update the available spots for our drop down menu."""
        spot_name = str(spot_name)
        if active:
            self.spot_chooser.addItem(spot_name)
            self.spot_data[spot_name] = {"feature_hashes": feature_hashes}
        else:
            try:
                del self.spot_data[spot_name]
            except KeyError: # For now, if we start the GUI at a "bad" time, ignore this step because the spot what never active
                pass
            self.spot_chooser.clear()
            for s in self.spot_data.keys():
                self.spot_chooser.addItem(s)

        self.update()

    @QtCore.pyqtSlot(str)
    def choose_spot(self, spot):
        """ Choose a spot to create feature widgets for and display feature data. """
        if hasattr(self, "container"):
            layout = self.container.layout()
            for w in self.feature_widgets.values():
                layout.removeWidget(w)
                w.deleteLater()
            self.feature_widgets = dict()
            self.chosen_spot = spot
            self.create_feature_widgets()

    @QtCore.pyqtSlot()
    def trigger_freeze_graphs(self):
        """ Set freeze flag for all feature widgets. """
        for fw in self.feature_widgets.values():
            fw.frozen = not fw.frozen

    def create_feature_widgets(self):
        """ Create all feature widgets inside a central QWidget container.
        
        Our main window has a container with all the graphs for all features of an exercise.
        We choose what spot to watch and build the container and all feature widgets only for that spot.
        """
        self.container = QWidget()
        self.container.setLayout(QGridLayout())

        spot_data = self.spot_data.get(self.chosen_spot)

        layout = self.container.layout()
        layout.addWidget(self.controls, 1, 1, 4, 1)

        # Feature specific data
        if spot_data:
            feature_hashes = spot_data["feature_hashes"]
            for i, h in enumerate(feature_hashes):
                feature_widget = FeatureGraphsWidget()
                self.feature_widgets[h] = feature_widget
                layout.addWidget(feature_widget, 1, 5 + i, 10, 1)

        self.setFixedHeight(900)
    
        self.setCentralWidget(self.container)

        self.update()
        

    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()\

    def stop(self):
        signal.signal(signal.SIGINT, interrupt)


class GUIHandler(QThread):
    """ Since we need to handle the GUI from another thread that our workerhandler, we create this QThread to start and stop the GUI."""
    def __init__(self):
        super().__init__()

    def run(self, gui):
        self.gui = gui
        gui.start()

    def stop(self):
        try:
            self.gui.close()
        except Exception:
            pass