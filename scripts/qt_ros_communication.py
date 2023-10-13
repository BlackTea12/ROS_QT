#!/usr/bin/env python3

import sys, os
from PyQt5.QtWidgets import *
from PyQt5 import uic
import threading
import rospy
import time
from std_msgs.msg import Float64

form_class = uic.loadUiType("/home/navifra1/develop_ws/src/qt_ros_control/resources/main_window.ui")[0]

class MyROSQtApp(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.init_data()

        self.setupUi(self)
        self.speed_percentage_label.setText("Speed Percentage = 0%")
        self.speed_percentage_slider.setValue(100)
        self.speed_percentage_slider.valueChanged.connect(self.speed_percent_pub)

        self.setWindowTitle('ROS Control GUI')
        self.show()

    def init_data(self):
        # data
        self.speed_percent_slider_data = Float64()

        # init node
        rospy.init_node('ros_gui_node', anonymous=True)

        # ros publishers
        self.pub_speed_percent = rospy.Publisher('/navifra/speed_percent', Float64, queue_size=1)

    def speed_percent_pub(self,value):
        # Update data in GUI
        self.speed_percentage_label.setText("Speed Percentage = " + str(value) + "%")
        
        # pub 
        self.speed_percent_slider_data.data = value
        self.pub_speed_percent.publish(self.speed_percent_slider_data)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyROSQtApp()
    sys.exit(app.exec_())   # run GUI until system exits
