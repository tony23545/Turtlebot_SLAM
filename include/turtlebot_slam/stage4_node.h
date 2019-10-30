#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/Point.h>
#include <brick_search/ImageResult.h>

class Stage4Node{
private:
	int image_msg_count_ = 0;

	// Image transport and subscriber
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_{};
	image_transport::Publisher processed_image_pub_{};
	ros::Publisher result_pub_{}; // publish red area center

public:
	Stage4Node(ros::NodeHandle& nh);
	void imageCallback(const sensor_msgs::ImageConstPtr& image_msg_ptr);
};