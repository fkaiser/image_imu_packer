#!/bin/bash

input_bagfile_=/newfs/bagfiles/2015-08-29-12-32-32.bag
VIO_inputdata_bagfile_=/newfs/bagfiles/image_imu_packages/2015-08-29-12-32-32_VIO.bag
VIO_outputdata_bagfile_=/newfs/bagfiles/image_imu_packages/Viooutput.bag
vio_inpute_pack=0
run_vio=1
record_vio=0
Vio_bag_rate_=0.5
Secs_in_bag_=12



# Source correct ros environment
source ~/catkin_ws/devel/setup.bash

if [ $vio_inpute_pack -ge 1 ]; then
# Pack IMU,Image data in custom message
rosrun img_imu_packer img_imu_packing.py --input_bag $input_bagfile_ --output_bag $VIO_inputdata_bagfile_
fi

if [ $run_vio -ge 1 ]; then
 
#Start vio_ros node
roslaunch vio_ros vio.launch &

# Start visualization node
roslaunch vio_ros vis_no_images.launch &
rosrun rqt_image_view rqt_image_view &

if [ $record_vio -ge 1 ]; then
rosbag record /vio_vis/rviz/robot_pose /vio_vis/rviz/robot_path /vio_vis/vio_vis -O & $VIO_outputdata_bagfile_
fi

# Sleep for certain amount such that node will have been started
sleep 4

# Start rosbag with VIO input data
rosbag play --pause --start=$Secs_in_bag_ --rate=$Vio_bag_rate_ $VIO_inputdata_bagfile_


fi
# Start recording of VIO data
#record_VIO_data $3
# Start rosbag file 
#rosbag play  










