#!/usr/bin/env python
# license removed for brevity
__author__ = 'fabian'

import argparse
import sys
import rosbag
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from vio_ros.msg import VioSensorMsg


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


for topic, msg, t in in_bag.read_messages(topics=[imu_topic_name_, cam_left_topic_name_, cam_right_topic_name_]):
	# Check whether left camera image has arrived	
	if topic == cam_left_topic_name_:
		tmp.left_image = msg
		cam_left_arrived_ = 1
		tmp.header = tmp.left_image.header
		temp_t = t
	# Check whether right camera image has arrived	
	if topic == cam_right_topic_name_:
		tmp.right_image = msg
		cam_right_arrived_ = 1
	# Check whether IMU data has arrived	
	if topic == imu_topic_name_:
		tmp.imu = msg
		imu_arrived_ = 1

	# Check whether image and IMU data is available 
	if  imu_arrived_ and cam_right_arrived_ and cam_left_arrived_:
		out_bag.write(vio_topic_name_, tmp, temp_t)
		imu_arrived_ = 0
		cam_left_arrived_ = 0
		cam_right_arrived_ = 0
		

in_bag.close()
out_bag.close()
