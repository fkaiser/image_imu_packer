#include "ros/ros.h"
#include "vio_ros/VioSensorMsg.h"
#include <sstream>
#include "sensor_msgs/Image.h"
#include <ros/callback_queue.h>
// This node sends service request frequently to get image at fixed frequency

class ImgIMUPacker{

public:
	ImgIMUPacker(): n_("/image_imu_packer"){
		// Get information for nodes
		ros::param::get("~cam_topicname_left",cam_topicname_left_);
		ros::param::get("~cam_topicname_right",cam_topicname_right_);
		ros::param::get("~imu_topicname",imu_topicname_);
		ros::param::get("~package_topicname",package_topicname_);
		printoutnameinfo();

		img_left_new_=false;
		img_right_new_=false;
		imu_new_=false;
		publish_ok_=false;
		// Subscribe to imgage of left camera
		subscribetoimages_IMU();
		// Advertise img imu package
		img_pub_=n_.advertise<vio_ros::VioSensorMsg>(package_topicname_, 1,this);
	}

	void printoutnameinfo(){
		std::cout<<"Read from parameter server:"<<std::endl;
		std::cout<<"Going to publish on the following name: "<<package_topicname_<<std::endl<< "Subscribed to left image topic: "<<cam_topicname_left_<<std::endl;
		std::cout<< "Subscribed to right image topic: "<<cam_topicname_right_<<std::endl;
		std::cout<< "Subscribed to imu topic: "<<imu_topicname_<<std::endl;
	}


	void subscribetoimages_IMU(){
		img_left_sub_ = n_.subscribe(cam_topicname_left_,1,&ImgIMUPacker::img_buffering_left,this);
		img_right_sub_ = n_.subscribe(cam_topicname_right_,1,&ImgIMUPacker::img_buffering_right,this);
		imu_sub_= n_.subscribe(imu_topicname_,1,&ImgIMUPacker::imu_buffering,this);

	}

	void img_buffering_left(const sensor_msgs::ImageConstPtr& msg){
		img_left_=*msg;
		img_left_new_=true;
	}
	void img_buffering_right(const sensor_msgs::ImageConstPtr& msg){
		img_right_=*msg;
		img_right_new_=true;
	}
	void imu_buffering(const sensor_msgs::Imu& msg){
		imu_=msg;
		imu_new_=true;
	}

void publish_Img_IMU_package()
{
	img_imu_package_.left_image=img_left_;
	img_imu_package_.right_image=img_right_;
	img_imu_package_.imu=imu_;
	publish_ok_=false;
	img_left_new_=false;
	img_right_new_=false;
	imu_new_=false;
	img_pub_.publish(img_imu_package_);
}
bool check_publish_package()
{

	if (img_right_new_ && img_left_new_&& imu_new_)
	{

		publish_ok_=true;
	}
	return publish_ok_;
}

private:
	ros::NodeHandle n_;
	ros::Publisher img_pub_;
	std::string service_name_;
	std::string topic_name_;
	std::string cam_topicname_left_;
	std::string cam_topicname_right_;
	std::string imu_topicname_;
	std::string package_topicname_;
	ros::Subscriber img_left_sub_;
	ros::Subscriber img_right_sub_;
	ros::Subscriber imu_sub_;
	sensor_msgs::Image img_left_;
	sensor_msgs::Image img_right_;
	sensor_msgs::Imu imu_;
	vio_ros::VioSensorMsg img_imu_package_;
	bool img_left_new_;
	bool img_right_new_;
	bool imu_new_;
	bool publish_ok_;

};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Image_sender_server");
	ImgIMUPacker img_imu_packing_point;
	ros::Rate R(40);
	while(ros::ok()){
		ros::spinOnce();
		if (img_imu_packing_point.check_publish_package()){
			img_imu_packing_point.publish_Img_IMU_package();
		}
		R.sleep();
	}

	return 0;
}
