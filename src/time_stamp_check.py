#!/usr/bin/env python
# license removed for brevity
__author__ = 'fabian'

import argparse
import sys
import numpy as np
import rosbag
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from vio_ros.msg import VioSensorMsg
from copy import deepcopy
import rospy
import sys

from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg

#QtGui.QApplication.setGraphicsSystem('raster')
app = QtGui.QApplication([])
#mw = QtGui.QMainWindow()
#mw.resize(800,800)

win = pg.GraphicsWindow(title="Basic plotting examples")
win.resize(1000,600)
win.setWindowTitle('pyqtgraph example: Plotting')

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)




if __name__ == "__main__":
    rosbag_input_name='/newfs/bagfiles/image_imu_packages/2015-08-29-12-32-32_VIO_input.bag'
    topic_name_='/vio_sensor'
    in_bag = rosbag.Bag(rosbag_input_name, 'r')
    vio_msg_timediff_data = [np.empty(5000)]
    bag_list=in_bag.read_messages(topics=[topic_name_])
    iter_image_left=0
    t_bag=[]
    t_msg=[]
    iter_indx=0
    for topic, msg, t in bag_list:
        t_bag_now=float(t.to_sec())
        t_msg_now=float(msg.header.stamp.secs+msg.header.stamp.nsecs/1e9)
        if iter_indx>0:
            t_bag.append(t_bag_now-t_bag_pre)
            t_msg.append(t_msg_now-t_msg_pre)
        t_bag_pre=t_bag_now
        t_msg_pre=t_msg_now
        iter_indx=iter_indx+1
        
    p1 = win.addPlot(title="Time difference messages",y=t_msg)
    win.nextRow()
    p2 = win.addPlot(title="Time difference bag",y=t_bag)
    
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
            
            
            
            