#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float64

class RQT_DATA:
    def __init__(self):
        # data
        self.speed_percent_slider_data = Float64()

        # flags
        self.b_update_speed_percent = False

    def ros_run(self):
        # init node
        # rospy.init_node('ros_gui_node', anonymous=True)

        # ros publishers
        self.pub_speed_percent = rospy.Publisher('/navifra/speed_percent', Float64, queue_size=1)
        
        # publish all data
        while not rospy.is_shutdown():
            self.PublishAll()
            time.sleep(0.1)

    def SaveSpeedPercent(self, data):
        self.speed_percent_slider_data.data = data
        self.b_update_speed_percent = True
    
    def PublishAll(self):
        if self.b_update_speed_percent == True:
            self.pub_speed_percent(self.speed_percent_slider_data)
            self.b_update_speed_percent = False