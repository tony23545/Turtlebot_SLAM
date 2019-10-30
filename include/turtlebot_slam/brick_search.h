#include <atomic>
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

class BrickSearch{
public:
	BrickSearch(ros::NodeHandle& nh);
	void mainLoop();

private:
	// Variables
	nav_msgs::OccupancyGrid map_{};
	cv::Mat map_image_{};
	std::atomic<bool> localised_{false};
	std::atomic<bool> brick_found_{false};
	int image_msg_count_ = 0;

	// Transform Listener
	tf2_ros::Buffer transform_buffer_{};
	tf2_ros::TransformListener transform_listener_{transform_buffer_};

	// subscribe to the AMCL pose to get covariance
	ros::Subscriber amcl_pose_sub_{};

	// velocity command publisher
	ros::Publisher cmd_vel_pub_{};

	// Image transport and subscriber
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_{};
	image_transport::Publisher processed_image_pub_{};

	// brick position and distance
	cv::Point p;
	double m00;

	// Action client
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_client_{"move_base", true};

	// Private methods
	void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg);
	void imageCallback(const sensor_msgs::ImageConstPtr& image_msg_ptr);
};