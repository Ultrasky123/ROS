from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
import sys


class MYGUI(QMainWindow):
    def __init__(self):
        super(MYGUI,self).__init__()
        self.setGeometry(400,200,600,400)
        self.setWindowTitle("CONTROL")
        self.initUI()

    
    def initUI(self):
        self.label = QtWidgets.QLabel(self)
        self.label.setText("sensor")
        self.label.move(300,50)

        self.bt1 = QtWidgets.QPushButton(self)
        self.bt1.setText("Reboot")
        self.bt1.clicked.connect(self.reboot)
        self.bt1.move(100,100)
    
    def reboot(self):
        self.label.setText ("reboot vehicle")
        self.update()
    
    #supaya text label gk ketimpa
    def update(self):
        self.label.adjustSize()

def reboot():
    print("reboot vehicle")

def window():
    app = QApplication(sys.argv)
    win = MYGUI()
    win.show()
    sys.exit(app.exec_())

window()