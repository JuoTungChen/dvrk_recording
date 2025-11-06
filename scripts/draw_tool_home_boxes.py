#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
import json
from sensor_msgs.msg import Joy
from PyQt5 import QtWidgets, QtCore, QtGui

class TransparentWindow(QtWidgets.QWidget):
    """
    A transparent PyQt5 window that can be resized and moved, and displays pedal press counts from ROS2 topics.
    """
    close_signal = QtCore.pyqtSignal()
    resize_signal = QtCore.pyqtSignal(int, int)
    update_pedal_count_signal = QtCore.pyqtSignal(int, int)

    def __init__(self, identifier, config=None, show_resize_grip=True, show_info=True):
        super().__init__()
        self.identifier = identifier
        self.config = config or {}
        self.show_resize_grip = show_resize_grip
        self.show_info = show_info

        self.pedal_count = 0
        self.pedal_bicoag_count = 0

        self._setup_ui()
        self.update_pedal_count_signal.connect(self.update_count_label)
        self.oldPos = self.pos()

    def _setup_ui(self):
        """Initializes the user interface of the window."""
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint | QtCore.Qt.WindowStaysOnTopHint)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)

        screen_number = self.config.get('screen_number', 0)
        x_pos = self.config.get('x_pos', 100)
        y_pos = self.config.get('y_pos', 100)
        width = self.config.get('width', 300)
        height = self.config.get('height', 200)
        self.set_window_geometry(screen_number, x_pos, y_pos, width, height)

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
        self.update_count_label(0, 0)

    def set_window_geometry(self, screen_number, x, y, width, height):
        """Sets the window's position and size on the specified screen."""
        try:
            screen = QtWidgets.QApplication.screens()[screen_number]
            screen_geometry = screen.geometry()
            self.setGeometry(screen_geometry.x() + x, screen_geometry.y() + y, width, height)
        except IndexError:
            print(f"Screen number {screen_number} is out of range. Using default screen.")
            screen = QtWidgets.QApplication.screens()[0]
            screen_geometry = screen.geometry()
            self.setGeometry(screen_geometry.x() + x, screen_geometry.y() + y, width, height)

    def update_info_label(self):
        """Updates the information label with the window's current position and size."""
        if self.show_info:
            self.info_label.setText(f"Pos: ({self.x()}, {self.y()}) Size: ({self.width()} x {self.height()})")
            self.info_label.adjustSize()
            self.info_label.move(10, self.height() - 30)

    def update_count_label(self, pedal, pedal_bicoag):
        """Updates the pedal count label."""
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
        self.save_config()

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.RightButton:
            self.close_signal.emit()

    def resizeEvent(self, event):
        if self.show_resize_grip:
            self.sizeGrip.move(self.width() - 20, self.height() - 20)
        self.resize_signal.emit(self.width(), self.height())
        self.update_info_label()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 0, 0, 128), 2))
        painter.drawRect(10, 10, self.width() - 20, self.height() - 40)

    def save_config(self):
        """Saves the window's current configuration to a file."""
        screen = QtWidgets.QApplication.screenAt(self.pos())
        screen_number = QtWidgets.QApplication.screens().index(screen)
        data = {
            'screen_number': screen_number,
            'x_pos': self.pos().x() - screen.geometry().x(),
            'y_pos': self.pos().y() - screen.geometry().y(),
            'width': self.width(),
            'height': self.height()
        }
        config_path = f'window_config_{self.identifier}.json'
        with open(config_path, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"Window configuration for {self.identifier} saved to '{config_path}'")

class PedalListenerNode(Node):
    """
    ROS2 Node that listens to pedal topics and updates the transparent windows.
    """
    def __init__(self, window1, window2):
        super().__init__('pedal_listener_node')
        self.window1 = window1
        self.window2 = window2

        self.create_subscription(Joy, "/footpedals/coag", self.coag_callback, 10)
        self.create_subscription(Joy, "/footpedals/bicoag", self.bicoag_callback, 10)
        self.create_subscription(Joy, "/footpedals/cam_minus", self.reset_callback, 10)

    def coag_callback(self, msg):
        if msg.buttons and msg.buttons[0] == 1:
            self.window1.pedal_count += 1
            self.window1.update_pedal_count_signal.emit(self.window1.pedal_count, self.window1.pedal_bicoag_count)

    def bicoag_callback(self, msg):
        if msg.buttons and msg.buttons[0] == 1:
            self.window1.pedal_bicoag_count += 1
            self.window1.update_pedal_count_signal.emit(self.window1.pedal_count, self.window1.pedal_bicoag_count)

    def reset_callback(self, msg):
        if msg.buttons and msg.buttons[0] == 1:
            self.window1.pedal_count = 0
            self.window1.pedal_bicoag_count = 0
            self.window1.update_pedal_count_signal.emit(self.window1.pedal_count, self.window1.pedal_bicoag_count)

def load_config(identifier):
    """Loads window configuration from a file."""
    try:
        with open(f'window_config_{identifier}.json', 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        return None

def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)

    config1 = load_config('window1')
    config2 = load_config('window2')

    window1 = TransparentWindow('window1', config=config1)
    window2 = TransparentWindow('window2', config=config2)
    
    window1.show()
    window2.show()

    # Sync resize between windows
    window1.resize_signal.connect(window2.resize)
    window2.resize_signal.connect(window1.resize)

    # Close all windows if one is closed
    window1.close_signal.connect(app.quit)
    window2.close_signal.connect(app.quit)

    pedal_listener_node = PedalListenerNode(window1, window2)
    
    # Use a QTimer to spin the ROS2 node
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(pedal_listener_node, timeout_sec=0.01))
    timer.start(50)  # 50ms timer

    exit_code = app.exec_()
    
    pedal_listener_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()