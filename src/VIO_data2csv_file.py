#!/usr/bin/env python
# license removed for brevity
import rosbag
import csv
import os
from vio_ros.msg import vio_vis



if __name__ == '__main__':
	tmp_message=vio_vis()
	pathname_='/newfs/bagfiles/output_VIO/'
	bagname_='start1.bag'
	filename_='start1.csv'
	total_path_bag_=os.path.join(pathname_,bagname_)
	topic_name_='/vio_vis/vio_vis'
	
	# Open bag file
	in_bag = rosbag.Bag(total_path_bag_,'r')
	path_csv_=os.path.join(pathname_,filename_)
	
	# Create csv file
	with open(path_csv_, 'w+') as csvfile:
		mycsv_writer = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
		# Look in bag file and save robot pose and time stamp
		#spamwriter.writerow(['Spam', 'Lovely Spam', 'Wonderful Spam'])
		for topic, msg, t in in_bag.read_messages(topics=[topic_name_]):
	
			mycsv_writer.writerow([msg.image.header.stamp.secs,msg.image.header.stamp.nsecs,msg.robot_pose.position.x,msg.robot_pose.position.y,msg.robot_pose.position.z,msg.robot_pose.orientation.x,msg.robot_pose.orientation.y,msg.robot_pose.orientation.z,msg.robot_pose.orientation.w])

	
	in_bag.close()
