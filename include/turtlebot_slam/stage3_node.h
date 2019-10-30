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

#include <brick_search/ImageResult.h>
#include <frontier_exploration/ExploreTaskActionGoal.h>
#include <actionlib_msgs/GoalID.h>

class Stage3Node{
private:
	// Transform Listener
	tf2_ros::Buffer transform_buffer_{};
	tf2_ros::TransformListener transform_listener_{transform_buffer_};

	// Action client
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_client_{"move_base", true};

	// Velocity command publisher
  	ros::Publisher cmd_vel_pub_{};
  	
	// goal and image result subscriber
	ros::Subscriber goalID_sub_{};
	ros::Subscriber result_sub_{};

	// store current goal ID to cancel exploration when needed
	actionlib_msgs::GoalID current_goalID;

	// exploration cancel publisher
	ros::Publisher cancel_pub_{};

	bool found_brick_, approached_;

public:
	Stage3Node(ros::NodeHandle& nh);
	void servoing(brick_search::ImageResult msg);
	void goalCallback(frontier_exploration::ExploreTaskActionGoal msg);
};