#include <atomic>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

class Stage2Node{
public:
	Stage2Node(ros::NodeHandle& nh);
	void mainLoop();

private:
	// Variables
	std::atomic<bool> localised_{false};

	// Transform Listener
	tf2_ros::Buffer transform_buffer_{};
	tf2_ros::TransformListener transform_listener_{transform_buffer_};

	// subscribe to the AMCL pose to get covariance
	ros::Subscriber amcl_pose_sub_{};

	// velocity command publisher
	ros::Publisher cmd_vel_pub_{};

	// Action client
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_client_{"move_base", true};

	// Private methods
	void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg);
};