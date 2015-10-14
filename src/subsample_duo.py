#!/usr/bin/env python
__author__ = 'nicolas'

import sys
import argparse
import rosbag
from sensor_msgs.msg import Imu
from duo3d_ros.msg import Duo3d

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Subsample a ros bag with DUO messages to have more IMU messages than image messages and correct the orientation and scaling of the IMU data',
                                     prog='rosrun ros_bag_tools subsample_duo.py')
    parser.add_argument('--input_bag', '-i', action='store', type=str, nargs=1, help='The source bag to read from. Must contain Duo3d combined topic')
    parser.add_argument('--subsample', '-s', action='store', type=int, nargs=1, default=4, help='The factor by which to subsample. The resulting bag will have this many more IMU messages than image msessages')
    parser.add_argument('--output_bag', '-o', action='store', type=str, nargs=1, default=['out.bag'], help='The destination bag')

    args = parser.parse_args()

    if args.input_bag is None:
        print(parser.parse_args(['-h']))
        sys.exit(-1)

    print('Input bag: {}'.format(args.input_bag[0]))

    in_bag = rosbag.Bag(args.input_bag[0], 'r')
    print(in_bag)

    print('\nWriting to bag: {}'.format(args.output_bag[0]))
    out_bag = rosbag.Bag(args.output_bag[0], 'w')

    subsample = args.subsample[0]
    print('Subsample: {}'.format(subsample))

    has_right = '/duo3d_camera/right/image_raw' in in_bag.get_type_and_topic_info()[1]

    imu_cnt = 0
    write_left = False
    write_right = False
    for topic, msg, t in in_bag.read_messages(topics=['/duo3d_camera/combined']):
        if imu_cnt % subsample == 0:
            left_image = msg.left_image
            shifted_header = msg.header
            time = shifted_header.stamp.nsecs + shifted_header.stamp.secs*1e9
            # time += (0.0150117606625+0.000657665377756+0.00119533281712-0.000487851613732)*1e9
            shifted_header.stamp.nsecs = time % 1e9
            shifted_header.stamp.secs = int(time/1e9)
            left_image.header = shifted_header
            right_image = msg.right_image
            right_image.header = msg.header
            out_bag.write('/duo3d_camera/left/image_raw', msg.left_image, t)
            out_bag.write('/duo3d_camera/right/image_raw', msg.right_image, t)

        imu_msg = Imu()
        imu_msg.angular_velocity.x = msg.imu.angular_velocity.x
        imu_msg.angular_velocity.y = -msg.imu.angular_velocity.y
        imu_msg.angular_velocity.z = msg.imu.angular_velocity.z

        imu_msg.linear_acceleration.x = msg.imu.linear_acceleration.x * 9.81
        imu_msg.linear_acceleration.y = -msg.imu.linear_acceleration.y * 9.81
        imu_msg.linear_acceleration.z = -msg.imu.linear_acceleration.z * 9.81

        imu_msg.header = msg.header

        out_bag.write('/duo3d_camera/cam_imu', imu_msg, t)

        imu_cnt += 1

    print('\nDone. Output bag:')
    print(out_bag)
    in_bag.close()
    out_bag.close()

