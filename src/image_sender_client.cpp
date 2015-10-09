#include "ros/ros.h"
#include "image_sender/SendImage.h"
#include <sstream>
#include "sensor_msgs/Image.h"
#include <ros/callback_queue.h>
// This node sends service request frequently to get image at fixed frequency

class ImgSenderClient{

public:
	ImgSenderClient(): n_("/request_client"){
		ros::param::get("~service_name",service_name_);
		ros::param::get("~topic_name",topic_name_);
		client_ = n_.serviceClient<image_sender::SendImage>(service_name_);
		img_pub_=n_.advertise<sensor_msgs::Image>(topic_name_, 1,this);
	}
	void ImgRequest()
	  {
		  if(client_.call(srv_)){
			  img_pub_.publish(srv_.response.img);
		  }
		  else
		  {
			  ROS_INFO_STREAM("Got no image");
		  }
	  }

private:
	ros::NodeHandle n_;
	ros::ServiceClient client_;
	image_sender::SendImage srv_;
	ros::Publisher img_pub_;
	std::string service_name_;
	std::string topic_name_;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Image_sender_server");
	ImgSenderClient img_client;
	ros::Rate R(1);
	while(ros::ok()){
		img_client.ImgRequest();
		R.sleep();
	}

	return 0;
}
