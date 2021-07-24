import sys
from typing import Callable
from src.layouts import VideoSelectionUi
from .data_manager import DataManager
from PyQt5.QtWidgets import QApplication, QMainWindow, QListWidgetItem, QDialog
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread, QObject

class VideoSelection(VideoSelectionUi, QObject):
    def __init__(self, data_manager = None):
        super().__init__()
        if data_manager is None:
            raise RuntimeError('This Gui Needs a data manager')
        self._app = QApplication(sys.argv)
        self._main_window = QMainWindow()
        self._gui = VideoSelectionUi()
        self._gui.setupUi(self._main_window)
        self.setupUi(self._main_window)
        self._data_manager : DataManager= data_manager
        self._index_callback : Callable = None
        self.change_video_button.clicked.connect(self.video_changed)

    def set_callback(self, callback : Callable):
        self._index_callback = callback

    def show(self):
        names = self._data_manager.get_camera_names_and_indices()
        print("names")
        for name in names:
            self.video_combobox.addItem(name[0], name[1])
        #self.video_combobox.addItems(names)
        self._main_window.show()
        sys.exit(self._app.exec_())

    def video_changed(self):
        index = self.video_combobox.currentIndex()
        data = self.video_combobox.itemData(index, Qt.UserRole)
        if self._index_callback is not None:
            self._index_callback(data)

def print_index(index):
    print("Test Index ", index)

if __name__=="__main__":
    a = VideoSelection()
    a.set_callback(print_index)
    a.show()
