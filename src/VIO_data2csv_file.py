#!/usr/bin/env python
# license removed for brevity
import argparse
import sys
import rosbag
import csv
import os
from vio_ros.msg import vio_vis



if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Saves output pose data from VIO in csv file',
                                     prog='rosrun img_imu_packer VIO_data2csv.py')
	parser.add_argument('--input_bag', '-i', action='store', type=str, nargs=1, help='The source bag to read from. Must contain vio_ros messages')
	parser.add_argument('--VIO_topic', '-v', action='store', type=str, nargs=1, default=['/vio_vis/vio_vis'], help='VIO package topic name')
	parser.add_argument('--csv_file', '-c', action='store', type=str, nargs=1, help='CSV file name with path')
	args = parser.parse_args()
	
	print('Input bag: {}'.format(args.input_bag[0]))
	in_bag = rosbag.Bag(args.input_bag[0], 'r')
	#print(in_bag)
	
	print('VIO input msg topic name: {}'.format(args.VIO_topic[0]))
	vio_topic_name_ = args.VIO_topic[0]
	
	print('Save data to: {}'.format(args.csv_file[0]))
	path_csv_ = args.csv_file[0]
	
	
	tmp_message=vio_vis()
	# Create csv file
	with open(path_csv_, 'w+') as csvfile:
		mycsv_writer = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
		
		# Look in bag file and save robot pose and time stamp
		#spamwriter.writerow(['Spam', 'Lovely Spam', 'Wonderful Spam'])
		for topic, msg, t in in_bag.read_messages(topics=[vio_topic_name_]):
		
			mycsv_writer.writerow([msg.image.header.stamp.secs,msg.image.header.stamp.nsecs,msg.robot_pose.position.x,msg.robot_pose.position.y,msg.robot_pose.position.z,msg.robot_pose.orientation.x,msg.robot_pose.orientation.y,msg.robot_pose.orientation.z,msg.robot_pose.orientation.w])

	
	in_bag.close()
