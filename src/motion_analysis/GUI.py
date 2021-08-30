#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the GUI for the motion analysis.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

from PyQt5.QtWidgets import QApplication, QGridLayout, QMainWindow, QPushButton, QWidget, QVBoxLayout
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

import rospy as rp

class MotionAnaysisGUI():
    def __init__(self):
        self.traces = dict()

        self.app = QtGui.QApplication([])
        self.main_window = QtGui.QMainWindow()
        self.main_window.resize(800,800)
        self.main_window.setWindowTitlte('gymy Motion Analysis GUI')

        self.win = pg.GraphicsWindow(title="Basic plotting examples")
        self.win.resize(1000,600)
        self.win.setWindowTitle('pyqtgraph example: Plotting')

        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)

        self.canvas = self.win.addPlot(title="Pytelemetry")

    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()

    def trace(self, name, dataset_x, dataset_y):
        if name in self.traces:
            self.traces[name].setData(dataset_x, dataset_y)
        else:
            self.traces[name] = self.canvas.plot(pen='y')


## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    gui = MotionAnaysisGUI()
    i = 0

    def update():
        global gui, i
        t = np.arange(0,3.0,0.01)
        s = sin(2 * pi * t + i)
        c = cos(2 * pi * t + i)
        gui.trace("sin",t,s)
        gui.trace("cos",t,c)
        i += 0.1

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)

    gui.start()
 

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motion Analysis GUI")
        self.setLayout(QVBoxLayout())
        btn1 = QPushButton("test")
        self.layout().addWidget(btn1)
        self.show()

        self.graphs = dict()

    def add_graph(self, name, data):
        """ Add a graph to the window, replacing the old graph, if it exists."""
        series = QLineSeries(self)
        for x, y in data:
            series.append(x, y)

        self.graphs["name"] = series


    def init_graphs(self):
        container = QWidget()
        container.setLayout(QGridLayout)



app = QApplication([])
mw = MainWindow()
app.exec_()
 
class MotionAnalysisGUI(QMainWindow):
    def __init__(self):
        super().__init__()
 
        self.setWindowTitle("PyQtChart Line")
        self.setGeometry(100,100, 680,500)
 
        self.show()

        self.chart =  QChart()
        self.chart.createDefaultAxes()
        self.chart.setAnimationOptions(QChart.SeriesAnimations)
        self.chart.setTitle("Line Chart Example")
 
        self.chart.legend().setVisible(True)
        self.chart.legend().setAlignment(Qt.AlignBottom)
 
        self.chartview = QChartView(self.chart)
        self.chartview.setRenderHint(QPainter.Antialiasing)
        self.chartview.update()

        self.setCentralWidget(self.chartview)

    def new_series(self, x):
 
        series = QLineSeries(self)
        series.append(0, 0 + x)
        series.append(1, 1 + x)
        self.chart.addSeries(series)
        
        self.chartview.update()

 
App = QApplication(sys.argv)
window = MotionAnalysisGUI()

for i in range(5):
    time.sleep(1)
    window.new_series(i)
    sys.exit(App.exec_())


