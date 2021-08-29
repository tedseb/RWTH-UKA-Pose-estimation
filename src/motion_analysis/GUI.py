#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the GUI for the motion analysis.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

from PyQt5.QtWidgets import QApplication, QMainWindow
import sys
from PyQt5.QtChart import QChart, QChartView, QLineSeries
from PyQt5.QtCore import QPointF
from PyQt5.QtGui import QPainter
from PyQt5.QtCore import Qt
import time
 
 
class MotionAnalysisGUI(QMainWindow):
    def __init__(self):
        super().__init__()
 
        self.setWindowTitle("PyQtChart Line")
        self.setGeometry(100,100, 680,500)
 
        self.show()

        self.chart = None
 
        self.create_linechart(0)
 
    def new_series(self, x):
 
        series = QLineSeries(self)
        series.append(0, 0 + x)
        series.append(1, 1 + x)
        
        return series

    def create_linechart(self, series):
        self.chart =  QChart()

        self.chart.addSeries(series)
        self.chart.createDefaultAxes()
        self.chart.setAnimationOptions(QChart.SeriesAnimations)
        self.chart.setTitle("Line Chart Example")
 
        self.chart.legend().setVisible(True)
        self.chart.legend().setAlignment(Qt.AlignBottom)
 
        chartview = QChartView(self.chart)
        chartview.setRenderHint(QPainter.Antialiasing)

        chartview.update()
 
        self.setCentralWidget(chartview)

 
App = QApplication(sys.argv)
window = MotionAnalysisGUI()

series = window.new_series(0)
window.create_linechart(series)

for i in range(5):
    time.sleep(1)
    series = window.new_series(i)
    window.chart.addSeries(series)


sys.exit(App.exec_())