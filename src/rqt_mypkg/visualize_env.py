import sys
import rospy
from sensor_msgs.msg import Joy
from PyQt5 import QtWidgets, QtCore, QtGui
import json
import os

class TransparentWindow(QtWidgets.QWidget):
    close_signal = QtCore.pyqtSignal()  # Signal to emit when closing
    resize_signal = QtCore.pyqtSignal(int, int)  # Signal for resizing

    def __init__(self, identifier, config=None, show_resize_grip=True, show_info=True):
        super().__init__()

        # Make the window frameless, transparent, and always on top
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint | QtCore.Qt.WindowStaysOnTopHint)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)

        # Set identifier for configuration purposes
        self.identifier = identifier
        self.show_resize_grip = show_resize_grip
        self.show_info = show_info
        self.pedal_count = 0
        self.pedal_bicoag_count = 0

        # Set initial properties from configuration
        self.screen_number = config.get('screen_number', 0) if config else 0
        self.set_window_geometry(
            config.get('x_pos', 100) if config else 100,
            config.get('y_pos', 100) if config else 100,
            config.get('width', 300) if config else 300,
            config.get('height', 200) if config else 200
        )

        # Add labels to display pedal counts
        self.pedal_label = QtWidgets.QLabel(f"Pedal Presses: {self.pedal_count}", self)
        self.pedal_bicoag_label = QtWidgets.QLabel(f"Bicoag Pedal Presses: {self.pedal_bicoag_count}", self)
        self.pedal_label.move(10, self.height() - 50)
        self.pedal_bicoag_label.move(10, self.height() - 30)
        self.pedal_label.setStyleSheet("QLabel { color : white; background-color: rgba(255, 0, 0, 100); }")
        self.pedal_bicoag_label.setStyleSheet("QLabel { color : white; background-color: rgba(255, 0, 0, 100); }")

        # Setup ROS Subscribers
        rospy.init_node('pedal_listener', anonymous=True)
        rospy.Subscriber("/footpedals/coag", Joy, self.get_pedal)
        rospy.Subscriber("/footpedals/bicoag", Joy, self.get_pedal_bicoag)
        rospy.Subscriber("/footpedals/cam_minus", Joy, self.reset_counts)

        # Tracking for dragging the window
        self.oldPos = self.pos()

    def set_window_geometry(self, x_pos, y_pos, width, height):
        screen = QtWidgets.QApplication.screens()[self.screen_number]
        screen_geometry = screen.geometry()
        self.setGeometry(screen_geometry.x() + x_pos, screen_geometry.y() + y_pos, width, height)

    def get_pedal(self, data):
        if data.buttons[0] == 1:
            self.pedal_count += 1
            self.pedal_label.setText(f"Pedal Presses: {self.pedal_count}")

    def get_pedal_bicoag(self, data):
        if data.buttons[0] == 1:
            self.pedal_bicoag_count += 1
            self.pedal_bicoag_label.setText(f"Bicoag Pedal Presses: {self.pedal_bicoag_count}")

    def reset_counts(self, data):
        if data.buttons[0] == 1:
            self.pedal_count = 0
            self.pedal_bicoag_count = 0
            self.pedal_label.setText(f"Pedal Presses: {self.pedal_count}")
            self.pedal_bicoag_label.setText(f"Bicoag Pedal Presses: {self.pedal_bicoag_count}")

    # Remaining methods (resizeEvent, paintEvent, etc.) are as previously defined

def load_config(identifier):
    try:
        with open(f'window_config_{identifier}.txt', 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        return None

def close_all():
    QtWidgets.QApplication.instance().quit()

def main():
    app = QtWidgets.QApplication(sys.argv)
    rospy.init_node('pedal_listener', anonymous=True)

    config1 = load_config('window1')
    config2 = load_config('window2')

    window1 = TransparentWindow('window1', config=config1)
    window2 = TransparentWindow('window2', config=config2)
    window1.show()
    window2.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
