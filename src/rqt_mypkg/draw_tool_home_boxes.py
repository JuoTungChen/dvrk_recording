#!/usr/bin/env python

import sys
import rospy
import json
from sensor_msgs.msg import Joy
from PyQt5 import QtWidgets, QtCore, QtGui

class TransparentWindow(QtWidgets.QWidget):
    close_signal = QtCore.pyqtSignal()
    resize_signal = QtCore.pyqtSignal(int, int)
    update_pedal_count_signal = QtCore.pyqtSignal(int, int)

    def __init__(self, identifier, config=None, show_resize_grip=True, show_info=True):
        super().__init__()

        # ROS Node Initialization
        rospy.init_node('pedal_listener', anonymous=True)

        # ROS Subscriptions
        self.sub_coag = rospy.Subscriber("/footpedals/coag", Joy, self.get_pedal)
        self.sub_bicoag = rospy.Subscriber("/footpedals/bicoag", Joy, self.get_pedal_bicoag)
        self.sub_cam_minus = rospy.Subscriber("/footpedals/cam_minus", Joy, self.reset_pedal_counts)

        self.pedal_count = 0
        self.pedal_bicoag_count = 0

        # Window setup
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint | QtCore.Qt.WindowStaysOnTopHint)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.identifier = identifier
        self.show_resize_grip = show_resize_grip
        self.show_info = show_info

        self.screen_number = config.get('screen_number', 0) if config else 0
        self.set_window_geometry(
            config.get('x_pos', 100) if config else 100,
            config.get('y_pos', 100) if config else 100,
            config.get('width', 300) if config else 300,
            config.get('height', 200) if config else 200
        )

        if self.show_info:
            self.info_label = QtWidgets.QLabel(self)
            self.info_label.setStyleSheet("QLabel { color : white; background-color: rgba(255, 0, 0, 100); }")
            self.update_info_label()

        if self.show_resize_grip:
            self.sizeGrip = QtWidgets.QSizeGrip(self)
            self.sizeGrip.setStyleSheet("QSizeGrip { background: rgba(255, 0, 0, 100); width: 20px; height: 20px; }")
            self.sizeGrip.move(self.width() - 20, self.height() - 20)

        self.count_label = QtWidgets.QLabel(self)
        self.count_label.setStyleSheet("QLabel { color : white; background-color: rgba(0, 100, 0, 100); }")
        self.count_label.move(10, 10)
        self.update_pedal_count_signal.connect(self.update_count_label)

        self.oldPos = self.pos()

    def set_window_geometry(self, x, y, width, height):
        screen = QtWidgets.QApplication.screens()[self.screen_number]
        screen_geometry = screen.geometry()
        self.setGeometry(screen_geometry.x() + x, screen_geometry.y() + y, width, height)

    def update_info_label(self):
        self.info_label.setText(f"Pos: ({self.x()}, {self.y()}) Size: ({self.width()} x {self.height()})")
        self.info_label.adjustSize()
        self.info_label.move(10, self.height() - 30)

    def get_pedal(self, data):
        if data.buttons[0] == 1:
            self.pedal_count += 1
            self.update_pedal_count_signal.emit(self.pedal_count, self.pedal_bicoag_count)

    def get_pedal_bicoag(self, data):
        if data.buttons[0] == 1:
            self.pedal_bicoag_count += 1
            self.update_pedal_count_signal.emit(self.pedal_count, self.pedal_bicoag_count)

    def reset_pedal_counts(self, data):
        if data.buttons[0] == 1:
            self.pedal_count = 0
            self.pedal_bicoag_count = 0
            self.update_pedal_count_signal.emit(self.pedal_count, self.pedal_bicoag_count)

    def update_count_label(self, pedal, pedal_bicoag):
        self.count_label.setText(f"Pedal: {pedal}, BiCoag: {pedal_bicoag}")
        self.count_label.adjustSize()


    def mousePressEvent(self, event):
        self.oldPos = event.globalPos()

    def mouseMoveEvent(self, event):
        if event.buttons() == QtCore.Qt.LeftButton:
            delta = QtCore.QPoint(event.globalPos() - self.oldPos)
            self.move(self.x() + delta.x(), self.y() + delta.y())
            self.oldPos = event.globalPos()
            self.update_info_label()

    def mouseDoubleClickEvent(self, event):
        # Save the current configuration to a file
        data = {
            'screen_number': self.screen_number,
            'x_pos': self.x(),
            'y_pos': self.y(),
            'width': self.width(),
            'height': self.height()
        }
        with open(f'window_config_{self.identifier}.txt', 'w') as f:
            json.dump(data, f)
        print(f"Window configuration for {self.identifier} saved to 'window_config_{self.identifier}.txt'")

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.RightButton:
            self.close_signal.emit()  # Emit the close signal when right-clicked

    def resizeEvent(self, event):
        if self.show_resize_grip:
            self.sizeGrip.move(self.width() - 20, self.height() - 20)
        self.resize_signal.emit(self.width(), self.height())  # Emit resize signal
        self.update_info_label()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 0, 0, 128), 2))
        painter.drawRect(10, 10, self.width() - 20, self.height() - 40)

    def closeEvent(self, event):
        self.close_signal.emit()

def load_config(identifier):
    try:
        with open(f'window_config_{identifier}.txt', 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        return None

def close_all():
    QtWidgets.QApplication.instance().quit()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    config1 = load_config('window1')
    config2 = load_config('window2')

    window1 = TransparentWindow('window1', config=config1)
    window2 = TransparentWindow('window2', config=config2)
    window1.show()
    window2.show()

    window1.resize_signal.connect(lambda w, h: window2.resize(w, h))
    window2.resize_signal.connect(lambda w, h: window1.resize(w, h))
    window1.close_signal.connect(close_all)
    window2.close_signal.connect(close_all)

    sys.exit(app.exec_())