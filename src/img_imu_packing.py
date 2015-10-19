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
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg


def give_indx_sorted(list_to_sort):
	# Sort time vector
	list_sorted=deepcopy(list_to_sort)
	list_sorted.sort()
	indexDict =dict([ (value, index) for index, value in enumerate(list_to_sort)])
	idx_sorted_list=[]
	for entry in list_sorted:
		idx_sorted_list.append(indexDict[entry])
 	
	return idx_sorted_list

def sort_list_dict(list_to_sort,idx_sorted_list):
	list_sorted=[]
	for indx_sorted in idx_sorted_list:
		list_sorted.append(list_to_sort[indx_sorted])
	return list_sorted
	

def imu_interpolate(t1,imu_data,imu_data_time):
	# Find to time instance left and right from t1 in imu_data
	index_closest=np.argmin([abs(x-t1) for x in imu_data_time])
	sign_closest=np.sign(imu_data_time[index_closest]-t1)
	if sign_closest>0:
		index_right_c=index_closest
		index_left_c=index_right_c
		while True:
			if (index_left_c==0 or np.sign(imu_data_time[index_left_c]-t1)<0):
				break
			index_left_c=index_left_c-1
	if sign_closest<0:
		index_left_c=index_closest
		index_right_c=index_left_c
		while True:
			if (index_right_c==len(imu_data_time)-1 or np.sign(imu_data_time[index_right_c]-t1)>0):
				break
			index_right_c=index_right_c+1
			
	if sign_closest==0:
		index_right_c=index_closest
		index_left_c=index_right_c
		
	imu_data_inter=deepcopy(imu_data[index_closest])
	imu_data_inter.linear_acceleration.x=lin_interpolate(imu_data[index_left_c].linear_acceleration.x,imu_data[index_right_c].linear_acceleration.x,imu_data_time[index_left_c],imu_data_time[index_right_c],t1)
	imu_data_inter.linear_acceleration.y=lin_interpolate(imu_data[index_left_c].linear_acceleration.y,imu_data[index_right_c].linear_acceleration.y,imu_data_time[index_left_c],imu_data_time[index_right_c],t1)
	imu_data_inter.linear_acceleration.z=lin_interpolate(imu_data[index_left_c].linear_acceleration.z,imu_data[index_right_c].linear_acceleration.z,imu_data_time[index_left_c],imu_data_time[index_right_c],t1)
	imu_data_inter.angular_velocity.x=lin_interpolate(imu_data[index_left_c].angular_velocity.x,imu_data[index_right_c].angular_velocity.x,imu_data_time[index_left_c],imu_data_time[index_right_c],t1)
	imu_data_inter.angular_velocity.y=lin_interpolate(imu_data[index_left_c].angular_velocity.y,imu_data[index_right_c].angular_velocity.y,imu_data_time[index_left_c],imu_data_time[index_right_c],t1)
	imu_data_inter.angular_velocity.z=lin_interpolate(imu_data[index_left_c].angular_velocity.z,imu_data[index_right_c].angular_velocity.z,imu_data_time[index_left_c],imu_data_time[index_right_c],t1)
	return imu_data_inter
		
def lin_interpolate(y1,y2,t1,t2,t_star):
	if t2-t1 != 0:
		y_star=y1+(y2-y1)/(t2-t1)*(t_star-t1)
	else:
		y_star=y1
	
	return y_star
		
			
			
	

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Packs IMU and Image data such that it can be read in VIO framework',
                                     prog='rosrun img_imu_packer img_imu_packing.py')
	parser.add_argument('--input_bag', '-i', action='store', type=str, nargs=1, help='The source bag to read from. Must IMU and Image data for left and right camera')
	parser.add_argument('--output_bag', '-o', action='store', type=str, nargs=1, default=['out.bag'], help='The destination bag')
	parser.add_argument('--cam_left_topic', '-l', action='store', type=str, nargs=1, default=['/mv_25000075/image_raw'], help='Left image topic name')
	parser.add_argument('--cam_right_topic', '-r', action='store', type=str, nargs=1, default=['/mv_25000060/image_raw'], help='Right image topic name')
	parser.add_argument('--IMU_topic', '-m', action='store', type=str, nargs=1, default=['/mavros/imu/data'], help='IMU topic name')
	parser.add_argument('--VIO_topic', '-v', action='store', type=str, nargs=1, default=['/vio_sensor'], help='VIO package topic name')
	
	args = parser.parse_args()
	
	if args.input_bag is None:
		print(parser.parse_args(['-h']))
		sys.exit(-1)		
	
	print('Left image topic name: {}'.format(args.cam_left_topic[0]))
	cam_left_topic_name_ = args.cam_left_topic[0]
	
	print('Right image topic name: {}'.format(args.cam_right_topic[0]))
	cam_right_topic_name_ = args.cam_right_topic[0]
	
	print('IMU topic name: {}'.format(args.IMU_topic[0]))
	imu_topic_name_ = args.IMU_topic[0]
	
	print('VIO output package topic name: {}'.format(args.VIO_topic[0]))
	vio_topic_name_ = args.VIO_topic[0]
	
	print('Input bag: {}'.format(args.input_bag[0]))
	in_bag = rosbag.Bag(args.input_bag[0], 'r')
	print(in_bag)
	
	print('\nWriting to bag: {}'.format(args.output_bag[0]))
	out_bag = rosbag.Bag(args.output_bag[0], 'w')

	tmp = VioSensorMsg()
	#out_bag = rosbag.Bag('/newfs/bagfiles/image_imu_packages/vio_bag_flying1.bag', 'w')
	#cam_left_topic_name_ = '/mv_25000075/image_raw'
	#cam_right_topic_name_ = '/mv_25000060/image_raw'
	#imu_topic_name_ = '/mavros/imu/data'
	
	cam_left_arrived_ = 0
	cam_right_arrived_ = 0
	imu_arrived_ = 0
	temp_t = 0
	imu_data_list=[]
	imu_time_list=[]
	image_left_list=[]
	image_left_time_list=[]
	image_right_list=[]
	image_right_time_list=[]
	bag_msg_t=[]
	
	bag_list=in_bag.read_messages(topics=[imu_topic_name_, cam_left_topic_name_, cam_right_topic_name_])
	iter_image_left=0
	for topic, msg, t in bag_list:
		# Check whether left camera image has arrived	
		if topic == cam_left_topic_name_:
			tmp.left_image = msg
			cam_left_arrived_ = 1
			tmp.header = tmp.left_image.header
			image_left_list.append(msg)
			image_left_time_list.append(float(msg.header.stamp.secs+msg.header.stamp.nsecs/1e9))
			bag_msg_t.append(t)
			#aux_time=deepcopy(bag_msg_t[0])
			#aux_time.set(msg.header.stamp.secs,msg.header.stamp.nsecs)
			#aux_time.set(float(msg.header.stamp.secs+tmp.imu.header.stamp.nsecs/1e9),0)
			temp_t = t
			#print "Time exact: %.4f" % image_left_time_list[iter_in]
			#print "Time something:%.4f" % aux_time.to_sec()
			if iter_image_left>1 and image_left_time_list[iter_image_left]-image_left_time_list[iter_image_left-1]<0.02:
				print("%.4f",image_left_time_list[iter_image_left]-image_left_time_list[iter_image_left-1])
			iter_image_left=iter_image_left+1
		# Check whether right camera image has arrived	
		if topic == cam_right_topic_name_:
			tmp.right_image = msg
			cam_right_arrived_ = 1
			image_right_list.append(msg)
			image_right_time_list.append(float(msg.header.stamp.secs+msg.header.stamp.nsecs*1e-9))
			
		# Check whether IMU data has arrived	
		if topic == imu_topic_name_:
			tmp.imu = msg
			imu_arrived_ = 1
			imu_data_list.append(msg)
			imu_time_list.append(float(msg.header.stamp.secs+msg.header.stamp.nsecs*1e-9))

		# Check whether image and IMU data is available 
		if  imu_arrived_ and cam_right_arrived_ and cam_left_arrived_:
			temp_t.set(tmp.left_image.header.stamp.secs,tmp.left_image.header.stamp.nsecs)
			
 			#out_bag.write(vio_topic_name_, tmp, temp_t)
			imu_arrived_ = 0
			cam_left_arrived_ = 0
			cam_right_arrived_ = 0

	# Sort lists with respect to their time
# 	imu_time_list_sorted=sort_list_dict(imu_time_list,give_indx_sorted(imu_time_list))
# 	imu_data_list_sorted=sort_list_dict(imu_data_list,give_indx_sorted(imu_time_list))
# 	image_right_time_list_sorted=sort_list_dict(image_right_time_list,give_indx_sorted(image_right_time_list))
# 	image_right_list_sorted=sort_list_dict(image_right_list,give_indx_sorted(image_right_time_list))
#  	image_left_time_list_sorted=sort_list_dict(image_left_time_list,give_indx_sorted(image_left_time_list))
#  	image_left_list_sorted=sort_list_dict(image_left_list,give_indx_sorted(image_left_time_list))

	imu_time_list_sorted=imu_time_list
	imu_data_list_sorted=imu_data_list
	image_right_time_list_sorted=image_right_time_list
	image_right_list_sorted=image_right_list
 	image_left_time_list_sorted=image_left_time_list
 	image_left_list_sorted=image_left_list


# Find start index where there are image from the left and right cameras
	if image_left_time_list_sorted[0]-image_right_time_list_sorted[0]<0:
		i_start=0
	else:
		tmp2=[abs(x-image_right_time_list_sorted[0]) for x in image_left_time_list_sorted]
		i_start=np.argmin(tmp2)
		
		# Go through image list and make packages
	current_time=deepcopy(bag_msg_t[0])
	t_bag_pre=t
	t_msg=[]
	iter_indx=0
	for i in range(i_start,len(image_left_time_list_sorted)-1):
		tmp=VioSensorMsg()
		tmp.left_image = image_left_list_sorted[i]	
		tmp.header = tmp.left_image.header
		current_time.set(tmp.left_image.header.stamp.secs,tmp.left_image.header.stamp.nsecs)
# 		temp_t.set(tmp.left_image.header.stamp.secs,tmp.left_image.header.stamp.nsecs)
		index_img_r=np.argmin([abs(x-image_left_time_list_sorted[i]) for x in image_right_time_list_sorted])
		tmp.right_image = image_right_list_sorted[index_img_r]
		tmp.imu=imu_interpolate(image_left_time_list_sorted[i],imu_data_list_sorted,imu_time_list_sorted)
		tmp.imu.header.stamp=tmp.left_image.header.stamp
		t_msg_now=float(tmp.header.stamp.secs+tmp.header.stamp.nsecs/1e9)
		if iter_indx>0:
			t_msg.append(t_msg_now-t_msg_pre)
		t_msg_pre=t_msg_now
		iter_indx=iter_indx+1	
		# Set one message
		out_bag.write(vio_topic_name_,deepcopy(tmp),deepcopy(current_time))

		t_new=0.5*(image_left_time_list_sorted[i+1]+image_left_time_list_sorted[i])
		t_new_sec=int(t_new)
 		t_new_nsec=int((t_new-t_new_sec)*1e9)
 		current_time.set(t_new_sec,t_new_nsec)
 		current_time=deepcopy(current_time)
 		t_msg_now=t_new
 		if iter_indx>0:
 			t_msg.append(t_msg_now-t_msg_pre)
 		t_msg_pre=t_msg_now
 		iter_indx=iter_indx+1
 		tmp=VioSensorMsg()
 		current_time=deepcopy(current_time)
 		tmp.imu=imu_interpokilate(t_new,imu_data_list_sorted,imu_time_list_sorted)
 		tmp.header.stamp.secs=t_new_sec
 		tmp.header.stamp.nsecs=t_new_nsec
 		tmp.imu.header.stamp=tmp.header.stamp
 		
 		tmp.imu.header.stamp=deepcopy(tmp.header.stamp)
 		# Set one message
 		
 		if t_msg[iter_indx-2]<0.045:
  			out_bag.write(vio_topic_name_,deepcopy(tmp),deepcopy(current_time))
  		else:
  			print('Dropped IMU message')
  			
  			

 		
# 		
		
	# Create vio input message that contains 2 times more IMU data then images
# 	for i in range(0,len())
# 			print()	

# 	print(image_right_time_list)

	in_bag.close()
	out_bag.close()
	
	app = QtGui.QApplication([])
#mw = QtGui.QMainWindow()
#mw.resize(800,800)

	win = pg.GraphicsWindow(title="Basic plotting examples")
	win.resize(1000,600)
	win.setWindowTitle('pyqtgraph example: Plotting')

# Enable antialiasing for prettier plots
	pg.setConfigOptions(antialias=True)
	p1 = win.addPlot(title="Time difference messages",y=t_msg)
	if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
		QtGui.QApplication.instance().exec_()

