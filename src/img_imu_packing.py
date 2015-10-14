#!/usr/bin/env python
# license removed for brevity
import rosbag
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from vio_ros.msg import VioSensorMsg

tmp=VioSensorMsg()
in_bag = rosbag.Bag('/newfs/bagfiles/2015-09-10-09-29-53.bag','r')
out_bag=rosbag.Bag('/newfs/bagfiles/image_imu_packages/vio_bag_flying1.bag','w')
cam_left_topic_name_='/mv_25000075/image_raw'
cam_right_topic_name_='/mv_25000060/image_raw'
cam_imu_topic_name_='/mavros/imu/data'
cam_left_arrived_=0
cam_right_arrived_=0
imu_arrived_=0
temp_t=0


for topic, msg, t in in_bag.read_messages(topics=[cam_imu_topic_name_,cam_left_topic_name_,cam_right_topic_name_]):
	# Check whether left camera image has arrived	
	if topic==cam_left_topic_name_:
		tmp.left_image=msg
		cam_left_arrived_=1
		tmp.header=tmp.left_image.header
		temp_t=t
	# Check whether right camera image has arrived	
	if topic==cam_right_topic_name_:
		tmp.right_image=msg
		cam_right_arrived_=1
	# Check whether IMU data has arrived	
	if topic==cam_imu_topic_name_:
		tmp.imu=msg
		imu_arrived_=1

	# Check whether image and IMU data is available 
	if  imu_arrived_ and cam_right_arrived_ and cam_left_arrived_:
		out_bag.write('/vio_sensor', tmp, temp_t)
		imu_arrived_=0
		cam_left_arrived_=0
		cam_right_arrived_=0
		

in_bag.close()
out_bag.close()
