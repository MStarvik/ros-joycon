import sys

from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QApplication, QLabel
from PyQt5.QtGui import QIcon, QPixmap

import rospy


class JoyconWindow(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwars)

        grid_layout = QGridLayout()
        self.setLayout(grid_layout)

        for x in range(3):
            for y in range(3):
                button = JoyconButton(str(str(3*x+y)))
                grid_layout.addWidget(button, x*2, y)
        
        self.setWindowTitle('Basic Grid Layout')


class JoyconButton(QPushButton):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setCheckable(True)
        self.toggle()
        self.clicked.connect(self)

    def __call__(self):
        print("button pushed")



if __name__ == "__main__":
    app = QApplication(sys.argv)

    joycon_window = JoyconWindow()
    joycon_window.show()
    
    sys.exit(app.exec_())