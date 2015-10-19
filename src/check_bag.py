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
import pyqtgraph as pg
import numpy as np
import time

if __name__ == "__main__":

    rospy.init_node("bag_checker")
	



    win = pg.GraphicsWindow()
    win.setGeometry(992, 0, 927, 560)
    win.setWindowTitle('Time stamp difference')
    vio_msg_timediff_plot = win.addPlot(title='Time difference messages')
    win.nextRow()
    rosbag_timediff_plot = win.addPlot(title='Time difference bag messages')


    vio_msg_timediff_plot.addLegend()
    rosbag_timediff_plo.addLegend()
  
    vio_msg_timediff_curves = [vio_msg_timediff_plot.plot(pen=(255, 0, 0), name='t_m')]
    rosbag_timediff = [rosbag_timediff_plot.plot(pen=(255, 0, 0), name='t_b')]
    vio_msg_timediff_data = [np.empty(100000)]
    rosbag_timediff_data = [np.empty(100000)]
    plot_idx = 0

    vis = Visualizer()
    vis.newData.connect(plotter)
    vis.start()

    # pg.QtGui.QApplication.exec_()
    pg.QtGui.QApplication.processEvents()
    while not rospy.is_shutdown():
        pg.QtGui.QApplication.processEvents()
        time.sleep(0.01)
    pg.QtGui.QApplication.closeAllWindows()
