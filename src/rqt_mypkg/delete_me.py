from PyQt5 import QtWidgets, QtCore, QtGui

class TransparentWindow(QtWidgets.QWidget):
    close_signal = QtCore.pyqtSignal()  # Signal to emit when closing

    def __init__(self, screen_number=0, x_pos=100, y_pos=100, show_resize_grip=True, show_info=True):
        super().__init__()

        # Make the window frameless and transparent
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)

        # Set window geometry based on screen number and coordinates
        self.set_window_geometry(screen_number, x_pos, y_pos)

        # Add a label to display the position and size, controlled by show_info flag
        self.show_info = show_info
        if self.show_info:
            self.info_label = QtWidgets.QLabel(self)
            self.info_label.setStyleSheet("QLabel { color : white; background-color: rgba(255, 0, 0, 100); }")
            self.update_info_label()

        # Enable size grip for resizing, controlled by show_resize_grip flag
        self.show_resize_grip = show_resize_grip
        if self.show_resize_grip:
            self.sizeGrip = QtWidgets.QSizeGrip(self)
            self.sizeGrip.setStyleSheet("QSizeGrip { background: rgba(255, 0, 0, 100); width: 20px; height: 20px; }")
            self.sizeGrip.move(self.width() - 20, self.height() - 20)

        # Tracking for dragging the window
        self.oldPos = self.pos()

    def set_window_geometry(self, screen_number, x_pos, y_pos):
        screen = QtWidgets.QApplication.screens()[screen_number]
        screen_geometry = screen.geometry()
        self.setGeometry(screen_geometry.x() + x_pos, screen_geometry.y() + y_pos, 300, 200)

    def update_info_label(self):
        if self.show_info:
            self.info_label.setText(f"Pos: ({self.x()}, {self.y()}) Size: ({self.width()} x {self.height()})")
            self.info_label.adjustSize()
            self.info_label.move(10, self.height() - 30)

    def mousePressEvent(self, event):
        self.oldPos = event.globalPos()

    def mouseMoveEvent(self, event):
        if event.buttons() == QtCore.Qt.LeftButton:
            delta = QtCore.QPoint(event.globalPos() - self.oldPos)
            self.move(self.x() + delta.x(), self.y() + delta.y())
            self.oldPos = event.globalPos()
            self.update_info_label()

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.RightButton:
            self.close_signal.emit()

    def resizeEvent(self, event):
        if self.show_resize_grip:
            self.sizeGrip.move(self.width() - 20, self.height() - 20)
        self.update_info_label()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 0, 0, 128), 2))
        painter.drawRect(10, 10, self.width() - 20, self.height() - 40)

    def closeEvent(self, event):
        self.close_signal.emit()

def close_all():
    QtWidgets.QApplication.instance().quit()

if __name__ == "__main__":
    app = QtWidgets.QApplication([])

    # Example: First window without resize grip and info, second with both
    show_resize_grip = show_info = True
    window1 = TransparentWindow(screen_number=0, x_pos=100, y_pos=100, 
                                show_resize_grip=show_resize_grip, show_info=show_info)
    window1.show()

    window2 = TransparentWindow(screen_number=0, x_pos=400, y_pos=300, 
                                show_resize_grip=show_resize_grip, show_info=show_info)
    window2.show()

    # Connect close signals to close_all function
    window1.close_signal.connect(close_all)
    window2.close_signal.connect(close_all)

    app.exec_()
