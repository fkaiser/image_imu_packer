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
			if (index_right_c==len(imu_data_time) or np.sign(imu_data_time[index_right_c]-t1)>0):
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
	if t2-t1>0:
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
	
	for topic, msg, t in bag_list:
	
		# Check whether left camera image has arrived	
		if topic == cam_left_topic_name_:
			tmp.left_image = msg
			cam_left_arrived_ = 1
			tmp.header = tmp.left_image.header
			image_left_list.append(msg)
			image_left_time_list.append(float(msg.header.stamp.secs+tmp.imu.header.stamp.nsecs*1e-9))
			bag_msg_t.append(t)
			temp_t = t
		# Check whether right camera image has arrived	
		if topic == cam_right_topic_name_:
			tmp.right_image = msg
			cam_right_arrived_ = 1
			image_right_list.append(msg)
			image_right_time_list.append(float(msg.header.stamp.secs+tmp.imu.header.stamp.nsecs*1e-9))
			
		# Check whether IMU data has arrived	
		if topic == imu_topic_name_:
			tmp.imu = msg
			imu_arrived_ = 1
			imu_data_list.append(msg)
			imu_time_list.append(float(msg.header.stamp.secs+tmp.imu.header.stamp.nsecs*1e-9))

		# Check whether image and IMU data is available 
		if  imu_arrived_ and cam_right_arrived_ and cam_left_arrived_:
			temp_t.set(tmp.left_image.header.stamp.secs,tmp.left_image.header.stamp.nsecs)
			
# 			out_bag.write(vio_topic_name_, tmp, temp_t)
			imu_arrived_ = 0
			cam_left_arrived_ = 0
			cam_right_arrived_ = 0

# 			print(diff_t)
	# Create 
#				print(counter_dropout)
#				print(max_t)

# Find start index where there are image from the left and right cameras
	if image_left_time_list[0]-image_right_time_list[0]<0:
		i_start=0
	else:
		tmp2=[abs(x-image_right_time_list[0]) for x in image_left_time_list]
		i_start=np.argmin(tmp2)
		
		# Go through image list and make packages
	current_time=deepcopy(bag_msg_t[0])
	for i in range(i_start,len(image_left_time_list)-1):
		tmp.left_image = image_left_list[i]	
		bag_msg_t[0].set(tmp.left_image.header.stamp.secs,tmp.left_image.header.stamp.nsecs)
		test_t=deepcopy(bag_msg_t[0])
# 		temp_t.set(tmp.left_image.header.stamp.secs,tmp.left_image.header.stamp.nsecs)
		index_img_r=np.argmin([abs(x-image_left_time_list[i]) for x in image_right_time_list])
		tmp.right_image = image_right_list[index_img_r]
		tmp.imu=imu_interpolate(image_left_time_list[i],imu_data_list,imu_time_list)
		# Set one message
# 		print(temp_t)
		out_bag.write(vio_topic_name_, tmp, bag_msg_t[0])
		print(current_time-bag_msg_t[0])
# 		t_new=0.5*(image_left_time_list[i+1]+image_left_time_list[i])
# 		t_new_sec=int(t_new)
# 		t_new_nsec=int((t_new-t_new_sec)*1e9)
# 		temp_t.set(t_new_sec,t_new_nsec)
# 		print(temp_t)
# 		tmp.imu=imu_interpolate(t_new,imu_data_list,imu_time_list)
# 		# Set one message
# 		out_bag.write(vio_topic_name_, tmp, temp_t)

		
	# Create vio input message that contains 2 times more IMU data then images
# 	for i in range(0,len())
# 			print()	

# 	print(image_right_time_list)

	in_bag.close()
	out_bag.close()
