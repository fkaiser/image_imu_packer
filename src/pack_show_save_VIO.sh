#!/bin/bash

bagname_=2015-08-29-12-32-32
input_bagfile_="/newfs/bagfiles/${bagname_}.bag"
VIO_inputdata_bagfile_="/newfs/bagfiles/image_imu_packages/${bagname_}_VIO_input.bag"
VIO_outputdata_bagfile_="/newfs/bagfiles/output_VIO/${bagname_}_VIO_output.bag"
CSV_filename_="/newfs/bagfiles/output_VIO/${bagname_}_VIO_output.csv"
Vio_bag_rate_=0.2
Secs_in_bag_=12

vio_inpute_pack=0
run_vio=1
record_vio=0
transfer_to_csv=0
kill_nodes=0



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
rosbag record /vio_vis/rviz/robot_pose /vio_vis/rviz/robot_path /vio_vis/vio_vis -O $VIO_outputdata_bagfile_ & 
fi

# Sleep for certain amount such that node will have been started
sleep 4

# Start rosbag with VIO input data
rosbag play --pause --start=$Secs_in_bag_ --rate=$Vio_bag_rate_ $VIO_inputdata_bagfile_
fi

# Kill all ros nodes
if [ $kill_nodes -ge 1 ]; then
sleep 2
rosnode kill --all
fi

if [ $transfer_to_csv -ge 1 ]; then
rosrun img_imu_packer VIO_data2csv_file.py --input_bag $VIO_outputdata_bagfile_ --csv_file $CSV_filename_

fi










