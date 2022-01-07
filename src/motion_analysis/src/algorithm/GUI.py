#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the GUI for the motion analysis.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

from os import error
from PyQt5.QtWidgets import QComboBox, QGridLayout, QLabel, QMainWindow, QProgressBar, QPushButton, QSizePolicy, QTextEdit, QWidget, QVBoxLayout
from pyqtgraph.Qt import QtGui, QtCore
import sys
from PyQt5.QtCore import QThread, pyqtSignal
import numpy as np
import pyqtgraph as pg
from pyqtgraph.graphicsItems.CurvePoint import CurveArrow

def clamp(x): # Used only for formating color strings
    return max(0, min(x, 255))

# We use a mixture of these two colors everywhere
GYMY_GREEN = (73, 173, 51)
GYMY_ORANGE = (247, 167, 11)

class FeatureGraphsWidget(QWidget):
    """ Implements a set of graphs that are displayed vertically on top of each other in our GUI to show us feature data."""
    update_user_data = pyqtSignal(np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray)
    update_static_data = pyqtSignal(np.ndarray, np.ndarray, np.ndarray, str, str)
    def __init__(self):
        super().__init__()
        self.update_user_data.connect(self._update_user_data)
        self.update_static_data.connect(self._update_static_data)

        self.setLayout(QGridLayout())
        layout = self.layout()

        # Initialize trajectories so that we can plot "something"
        self._user_trajectory_x = np.array([0])
        self._user_trajectory_y = np.array([0])
        self._discrete_user_trajectory_x = np.array([0])
        self._discrete_user_trajectory_y = np.array([0])
        self._filtered_user_trajectory_x = np.array([0])
        self._filtered_user_trajectory_y = np.array([0])
        self._filtered_reference_trajectory_x = np.array([0])
        self._filtered_reference_trajectory_y = np.array([0])
        self._errors_x = np.array([0])
        self._errors_y = np.array([0])
        self._progress_vector_x = np.array([0, 1])
        self._progress_vector_y = np.array([0, 0])
        self._prediction = 0

        # We use one plot widget per trajectory
        self.user_trajectory = pg.PlotWidget(title="user_trajectory")
        self.filtered_user_trajectory = pg.PlotWidget(title="filtered_user_trajectory")
        self.discrete_user_trajectory = pg.PlotWidget(title="discrete_user_trajectory")
        self.discrete_reference_trajectory = pg.PlotWidget(title="discrete_reference_trajectory")
        self.filtered_reference_trajectory = pg.PlotWidget(title="filtered_reference_trajectory")
        self.reference_trajectory = pg.PlotWidget(title="reference_trajectory")
        self.errors = pg.PlotWidget(title="errors")
        self.progress_vector = pg.PlotWidget(title="progress_vector")
        self.progress_vector.setXRange(-1, 1)
        self.progress_vector.setYRange(-1, 1)
        self.progress_vector.setAspectLocked()
        self.feature_progression = pg.PlotWidget(title="feature_progression")

        # For each plot widget, we initialize a curve
        self.user_trajectory_curve = pg.PlotCurveItem([0, 0])
        self.filtered_user_trajectory_curve = pg.PlotCurveItem([0, 0])
        self.reference_trajectory_curve = pg.PlotCurveItem([0, 0])
        self.discrete_user_trajectory_curve = pg.PlotCurveItem([0, 0])
        self.filtered_reference_trajectory_curve = pg.PlotCurveItem([0, 0])
        self.discrete_reference_trajectory_curve = pg.PlotCurveItem([0, 0])
        self.errors_curve = pg.PlotCurveItem([0, 0])
        self.progress_vector_curve = pg.PlotCurveItem([0, 0])

        self.user_trajectory.addItem(self.user_trajectory_curve)
        self.filtered_user_trajectory.addItem(self.filtered_user_trajectory_curve)
        self.reference_trajectory.addItem(self.reference_trajectory_curve)
        self.discrete_user_trajectory.addItem(self.discrete_user_trajectory_curve)
        self.filtered_reference_trajectory.addItem(self.filtered_reference_trajectory_curve)
        self.discrete_reference_trajectory.addItem(self.discrete_reference_trajectory_curve)
        self.errors.addItem(self.errors_curve)
        self.progress_vector.addItem(self.progress_vector_curve)

        self.feature_index_pointer = CurveArrow(self.discrete_reference_trajectory_curve, 0)
        self.feature_index_pointer.setStyle(angle=270, headWidth=5, pen=GYMY_ORANGE, brush= GYMY_ORANGE)

        self.name_label = QLabel(self)
        self.name_label.setText("Feature n")

        self.type_label = QLabel(self)
        self.type_label.setText("Type: ")

        self.spec_label = QTextEdit(self)
        self.spec_label.setReadOnly(True)
        self.spec_label.setText("Spec: ")

        layout.addWidget(self.name_label, 0, 0, 4, 1)
        layout.addWidget(self.progress_vector, 4, 0, 1, 1)
        layout.addWidget(self.user_trajectory, 5, 0, 4, 1)
        layout.addWidget(self.filtered_user_trajectory, 9, 0, 4, 1)
        layout.addWidget(self.discrete_user_trajectory, 13, 0, 4, 1)
        layout.addWidget(self.discrete_reference_trajectory, 17, 0, 4, 1)
        layout.addWidget(self.filtered_reference_trajectory, 21, 0, 4, 1)
        layout.addWidget(self.reference_trajectory, 25, 0, 4, 1)
        layout.addWidget(self.errors, 29, 0, 4, 1)
        layout.addWidget(self.type_label, 33, 0, 4, 1)
        layout.addWidget(self.spec_label, 37, 0, 4, 1)

        # Blocks all updates of plots
        self.frozen = False

        self.thread = QThread()

        # This timer saves us resources, because we do not update plots lots, but rather with 25 fps
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(1000 / 25)

        # Since we do not always re-plot reference plot data, we need to keep track of wether it has already been set
        self.reference_plot_data_set = False

    def hasHeightForWidth(self):
        """Always returns true in order to make pyQT ask for this widget hight for width."""
        return True

    def heightForWidth(self, w):
        """Return width times 3, which is the height that we choose for our widget"""
        return w * 3

    @QtCore.pyqtSlot(np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray)
    def _update_user_data(self, user_trajectory, filtered_user_trajectory, discrete_user_trajectory, errors, progress_vector, prediction):
        """ Set for our user trajectory, progress and errors arrays, but do not call setData already."""
        if self.frozen:
            return

        self._user_trajectory_x = np.array(range(len(user_trajectory)))
        self._user_trajectory_y = user_trajectory

        self._filtered_user_trajectory_x = np.array(range(len(filtered_user_trajectory)))
        self._filtered_user_trajectory_y = filtered_user_trajectory
        
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
            self.filtered_user_trajectory_curve.setData(self._filtered_user_trajectory_x, self._filtered_user_trajectory_y)
            self.feature_index_pointer.setIndex(self._prediction)
            self.errors_curve.setData(self._errors_x, self._errors_y)
            # TODO: draw vector here? https://stackoverflow.com/questions/44246283/how-to-add-a-arrow-head-to-my-line-in-pyqt4
            self.progress_vector_curve.setData(self._progress_vector_x, self._progress_vector_y, pen = GYMY_GREEN)
        except IndexError as e: # This occurs if our progress points to an index higher than our trajectory length
            logy.error_throttle(e)

    @QtCore.pyqtSlot(np.ndarray, np.ndarray, np.ndarray, str, str)
    def _update_static_data(self, reference_trajectory, filtered_reference_trajectory, discrete_reference_trajectory, feature_type, feature_spec):
        """ Call setData for our reference trajectory plots."""
        if self.frozen:
            return

        self.type_label.setText("Type: " + str(feature_type))
        self.spec_label.setText("Spec: " + str(feature_spec))

        reference_trajectory_x = np.array(range(len(reference_trajectory)))
        reference_trajectory_y = reference_trajectory 
        self.reference_trajectory_curve.setData(reference_trajectory_x, reference_trajectory_y)

        filtered_reference_trajectory_x = np.array(range(len(filtered_reference_trajectory)))
        filtered_reference_trajectory_y = np.array(filtered_reference_trajectory)
        self.filtered_reference_trajectory_curve.setData(filtered_reference_trajectory_x, filtered_reference_trajectory_y)

        discrete_reference_trajectory_x = np.array(range(len(discrete_reference_trajectory)))
        discrete_reference_trajectory_y = discrete_reference_trajectory
        self.discrete_reference_trajectory_curve.setData(discrete_reference_trajectory_x, discrete_reference_trajectory_y)

        self.reference_plot_data_set = True


class MotionAnaysisGUI(QMainWindow):
    """
    The MotionAnalysisGUI contains several widgets. One overall widget that displays the overall progress. And on top one widget per feature.
    The widgets are updated for every step of the motion anaylsis by a worker thread.
    The MotionAnalysisGUI furthermore offers a station selection field and a freeze button which freezes all graphs for closer inspection.
    """
    update_signal = pyqtSignal()
    update_overall_data_signal = pyqtSignal(float, float, np.ndarray, int, int)
    def __init__(self):
        self.app = QtGui.QApplication([])
        super().__init__()
        self.update_signal.connect(self.update)
        self.update_overall_data_signal.connect(self._update_overall_data)

        self.controls = QWidget(self)
        self.controls.setLayout(QVBoxLayout())
        controls_layout = self.controls.layout()

        self.overall_progress_vector_widget = pg.PlotWidget(title="overall_progress_vector")
        self.overall_progress_vector_widget.setAspectLocked()

        self.score_widget = QProgressBar(self)
        self.score_widget.setMaximum(100)
        self.score_widget.setFormat('Current repetition score: %p%')

        self.last_score_widget = QProgressBar(self)
        self.last_score_widget.setMaximum(100)
        self.last_score_widget.setFormat('Last repetition score: %p%')

        self.progress_widget = QProgressBar(self)
        self.progress_widget.setMaximum(100)
        self.progress_widget.setFormat('Repetition progress: %p%')
        
        hex_color = "#{0:02x}{1:02x}{2:02x}".format(clamp(int(GYMY_GREEN[0])), clamp(int(GYMY_GREEN[1])), clamp(int(GYMY_GREEN[2])))
        style = "QProgressBar::chunk \
            {{background-color: {};}} \
            text-align: center;".format(hex_color)
        self.score_widget.setStyleSheet(style)
        self.last_score_widget.setStyleSheet(style)
        self.progress_widget.setStyleSheet(style)
        
        self.alignment_widget = QProgressBar(self)
        self.alignment_widget.setMaximum(100)
        self.alignment_widget.setFormat('Features alignment: %p%')
        # self.overall_errors_widget = pg.PlotWidget(title="overall_errors")

        label1 = QLabel(self.controls)
        label1.setText("Aggregated Data")
        controls_layout.addWidget(label1)
        
        controls_layout.addWidget(self.overall_progress_vector_widget)
        # controls_layout.addWidget(self.overall_errors_widget)
        controls_layout.addWidget(self.score_widget)
        controls_layout.addWidget(self.last_score_widget)
        controls_layout.addWidget(self.progress_widget)
        controls_layout.addWidget(self.alignment_widget)

        self.overall_progress_vector_widget.setXRange(-1, 1)
        self.overall_progress_vector_widget.setYRange(-1, 1)
        self.overall_progress_vector_widget.setAspectLocked()

        self.overall_progress_vector_curve = pg.PlotCurveItem([0, 0])
        # self.overall_errors_curve = pg.PlotCurveItem([0, 0])

        self.overall_progress_vector_widget.addItem(self.overall_progress_vector_curve)
        self.overall_progress_vector_curve.setData(np.array([0, 1]), np.array([0, 0]), pen = GYMY_GREEN)
        # self.overall_errors_widget.addItem(self.overall_errors_curve)

        label2 = QLabel(self.controls)
        label2.setText("Choose Spot:")

        self.spot_chooser = QComboBox(self)
        self.spot_chooser.setEditable(False)
        self.spot_chooser.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        self.spot_chooser.currentTextChanged.connect(self.choose_spot)

        verticalSpacer = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        controls_layout.addItem(verticalSpacer)

        controls_layout.addWidget(label2)
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

        self.setFixedWidth(300)
        
        self.widgets_frozen = False

        self.show()

    @QtCore.pyqtSlot(float, float, np.ndarray, int, int)
    def _update_overall_data(self, progress, alignment, progress_alignment_vector, score, last_score):
        self._progress_vector_x = np.array([0, progress_alignment_vector[0]])
        self._progress_vector_y = np.array([0, progress_alignment_vector[1]])

        # Set alignment widget
        pen_color = tuple(np.array(GYMY_ORANGE) * (1 - alignment) + np.array(GYMY_GREEN) * alignment)
        self.overall_progress_vector_curve.setData(self._progress_vector_x, self._progress_vector_y, pen = pen_color)
        hex_color = "#{0:02x}{1:02x}{2:02x}".format(clamp(int(pen_color[0])), clamp(int(pen_color[1])), clamp(int(pen_color[2])))
        style = "QProgressBar::chunk \
            {{background-color: {};}} \
            text-align: center;".format(hex_color)
        self.alignment_widget.setStyleSheet(style)
        self.alignment_widget.setValue(alignment * 100)

        # Set repetition score
        pen_color = tuple(np.array(GYMY_ORANGE) * (1 - score/100) + np.array(GYMY_GREEN) * score/100)
        hex_color = "#{0:02x}{1:02x}{2:02x}".format(clamp(int(pen_color[0])), clamp(int(pen_color[1])), clamp(int(pen_color[2])))
        style = "QProgressBar::chunk \
            {{background-color: {};}} \
            text-align: center;".format(hex_color)
        self.score_widget.setStyleSheet(style)
        self.score_widget.setValue(score)

        # Set last repeitition score
        pen_color = tuple(np.array(GYMY_ORANGE) * (1 - last_score/100) + np.array(GYMY_GREEN) * last_score/100)
        hex_color = "#{0:02x}{1:02x}{2:02x}".format(clamp(int(pen_color[0])), clamp(int(pen_color[1])), clamp(int(pen_color[2])))
        style = "QProgressBar::chunk \
            {{background-color: {};}} \
            text-align: center;".format(hex_color)
        self.last_score_widget.setStyleSheet(style)
        self.last_score_widget.setValue(last_score)


        self.progress_widget.setValue(progress * 100)


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
        self.alignment_widget.setUpdatesEnabled(self.widgets_frozen)
        self.overall_progress_vector_widget.setUpdatesEnabled(self.widgets_frozen)
        self.score_widget.setUpdatesEnabled(self.widgets_frozen)
        self.last_score_widget.setUpdatesEnabled(self.widgets_frozen)
        self.progress_widget.setUpdatesEnabled(self.widgets_frozen)
        self.widgets_frozen = not self.widgets_frozen

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
                feature_widget.name_label.setText("Feature " + str(i))
                self.feature_widgets[h] = feature_widget
                layout.addWidget(feature_widget, 1, 5 + i, 10, 1)
            self.setFixedWidth(400 * (1 + (len(self.spot_data.get(self.chosen_spot).get("feature_hashes") or 0))))
            self.setFixedHeight(1200)
        else:
            self.setFixedWidth(300)
            self.setFixedHeight(400)
    
        self.setCentralWidget(self.container)

        self.update()
        
    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()\


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