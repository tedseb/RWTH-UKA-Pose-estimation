# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/basti/git/trainerai-core/src/infrastructure/station_manager/layouts/station_selection_ui.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(765, 327)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setMinimumSize(QtCore.QSize(100, 0))
        self.label.setMaximumSize(QtCore.QSize(100, 16777215))
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.station_combobox = QtWidgets.QComboBox(self.centralwidget)
        self.station_combobox.setMinimumSize(QtCore.QSize(300, 0))
        self.station_combobox.setCurrentText("")
        self.station_combobox.setObjectName("station_combobox")
        self.horizontalLayout.addWidget(self.station_combobox)
        self.activate_station_button = QtWidgets.QPushButton(self.centralwidget)
        self.activate_station_button.setMinimumSize(QtCore.QSize(130, 0))
        self.activate_station_button.setObjectName("activate_station_button")
        self.horizontalLayout.addWidget(self.activate_station_button)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem1)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setMinimumSize(QtCore.QSize(100, 0))
        self.label_2.setMaximumSize(QtCore.QSize(100, 16777215))
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_2.addWidget(self.label_2)
        self.exercise_edit = QtWidgets.QLineEdit(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.exercise_edit.sizePolicy().hasHeightForWidth())
        self.exercise_edit.setSizePolicy(sizePolicy)
        self.exercise_edit.setObjectName("exercise_edit")
        self.horizontalLayout_2.addWidget(self.exercise_edit)
        spacerItem2 = QtWidgets.QSpacerItem(150, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem2)
        self.activate_exercise_button = QtWidgets.QPushButton(self.centralwidget)
        self.activate_exercise_button.setMinimumSize(QtCore.QSize(130, 0))
        self.activate_exercise_button.setObjectName("activate_exercise_button")
        self.horizontalLayout_2.addWidget(self.activate_exercise_button)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem3)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setMinimumSize(QtCore.QSize(100, 0))
        self.label_3.setMaximumSize(QtCore.QSize(100, 16777215))
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_3.addWidget(self.label_3)
        self.weight_edit = QtWidgets.QLineEdit(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.weight_edit.sizePolicy().hasHeightForWidth())
        self.weight_edit.setSizePolicy(sizePolicy)
        self.weight_edit.setMinimumSize(QtCore.QSize(100, 0))
        self.weight_edit.setMaximumSize(QtCore.QSize(100, 16777215))
        self.weight_edit.setDragEnabled(False)
        self.weight_edit.setReadOnly(True)
        self.weight_edit.setObjectName("weight_edit")
        self.horizontalLayout_3.addWidget(self.weight_edit)
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_3.addWidget(self.label_4)
        self.doubleSpinBox = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.doubleSpinBox.setProperty("value", 0.5)
        self.doubleSpinBox.setObjectName("doubleSpinBox")
        self.horizontalLayout_3.addWidget(self.doubleSpinBox)
        spacerItem4 = QtWidgets.QSpacerItem(55, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem4)
        self.weight_detection_button = QtWidgets.QPushButton(self.centralwidget)
        self.weight_detection_button.setMinimumSize(QtCore.QSize(130, 0))
        self.weight_detection_button.setObjectName("weight_detection_button")
        self.horizontalLayout_3.addWidget(self.weight_detection_button)
        spacerItem5 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem5)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.output_text = QtWidgets.QTextEdit(self.centralwidget)
        self.output_text.setObjectName("output_text")
        self.verticalLayout.addWidget(self.output_text)
        spacerItem6 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem6)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 765, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Station Selection"))
        self.label.setText(_translate("MainWindow", "Select Station:"))
        self.activate_station_button.setText(_translate("MainWindow", "Activate Station"))
        self.label_2.setText(_translate("MainWindow", "exercise id:     "))
        self.exercise_edit.setText(_translate("MainWindow", "105"))
        self.activate_exercise_button.setText(_translate("MainWindow", "Start Exercise"))
        self.label_3.setText(_translate("MainWindow", "weight:"))
        self.weight_edit.setText(_translate("MainWindow", "0"))
        self.label_4.setText(_translate("MainWindow", "Seconds"))
        self.weight_detection_button.setText(_translate("MainWindow", "weight detection"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
