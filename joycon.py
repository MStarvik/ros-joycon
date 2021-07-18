import sys

from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QApplication, QLabel, QSizePolicy
from PyQt5.QtGui import QIcon, QPixmap

import rospy
from std_srvs.srv import Trigger


class JoyconWindow(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # self.setStyleSheet("background-color: #11100e;")
        self.setStyleSheet(
            "QWidget {"
                "background-color: #1b1916;"
            "}"
        )
        self.showFullScreen()

        grid_layout = QGridLayout()
        grid_layout.setSpacing(15);
        self.setLayout(grid_layout)

        for x in range(2):
            for y in range(4):
                button = JoyconButton("service {}".format(str(4*x+y)), self)
                button.set_service("service_{}".format(4*x+y))
                grid_layout.addWidget(button, y, x*2)

        label = QLabel(self)
        pixmap = QPixmap("icons/ascend_logo.png")
        pixmap = pixmap.scaledToHeight(320)
        label.setPixmap(pixmap)
        grid_layout.addWidget(label, 0, 1, 4, 1)
        
        self.setWindowTitle('ROS joycon')
        self.setWindowIcon(QIcon("icons/ascend_logo.png"))


class JoyconButton(QPushButton):
    def __init__(self, *args, service=None, **kwargs):
        super().__init__(*args, **kwargs)

        # self.setStyleSheet("border: red 5px solid;")
        # self.setStyleSheet("background-color: #1b1916")
        # self.setStyleSheet("color: #e4e4e4")

        self.setStyleSheet(
            "QPushButton {"
                "background-color: #1b1916;"
                # "background-color: #11100e;"
                "color: #e4e4e4;"
                # "color: #757575;"
                # "color: #f09b36;"
                "border: 3px solid #e4e4e4;"
                # "border: 2px solid #757575;"
                "border-radius: 8px;"
                "font-size: 24pt;"
            "}"
            "QPushButton::pressed {"
                "background-color: #e4e4e4;"
                # "background-color: #757575;"
                "color: #11100e;"
            "}"
        )
        

        self.service = service
        self.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding))
        self.clicked.connect(self)

    def __call__(self):
        if self.service:
            try:
                rospy.ServiceProxy(self.service, Trigger)()
            except rospy.ServiceException as e:
                print("Service call failed: {}".format(e))

    def set_service(self, service):
        self.service = service


if __name__ == "__main__":
    app = QApplication(sys.argv)

    joycon_window = JoyconWindow()
    joycon_window.show()
    
    sys.exit(app.exec_())
