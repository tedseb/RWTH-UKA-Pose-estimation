from PyQt5 import QtWidgets, uic
import sys

class Mainwindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__() # Call the inherited classes __init__ method
        uic.loadUi('poc.ui', self) # Load the .ui file

        self.statusbar.showMessage('Hello World')
        self.show() # Show the GUI


if __name__== '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = Mainwindow()
    app.exec_()