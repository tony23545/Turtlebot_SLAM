#include "brick_search/stage2_node.h"

Stage2Node::Stage2Node(ros::NodeHandle& nh){
	// wait for the transform to become available
	ROS_INFO("Waiting for transform from \"map\" to \"base_link\"");
	while(ros::ok() && !transform_buffer_.canTransform("map", "base_link", ros::Time(0.))){
		ros::Duration(0.1).sleep();
	}
	ROS_INFO("Transform available!");

	// subscribe to "amcl_pose" to get pose covariance
	amcl_pose_sub_ = nh.subscribe("amcl_pose", 1, &Stage2Node::amclPoseCallback, this);

	// advertise "cmd_vel" publisher to control TurtleBot manually
	cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);

	// action client for "move_base"
	ROS_INFO("Waiting for \"move_base\" action...");
	move_base_action_client_.waitForServer();
	ROS_INFO("\"move_base\" action available!");

	// reinitialise AMCL
	ros::ServiceClient global_localization_service_client = nh.serviceClient<std_srvs::Empty>("global_localization");
	std_srvs::Empty srv{};
	global_localization_service_client.call(srv);
}

void Stage2Node::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg){
	// check coveriance
	double frobenius_norm = 0;

	for(const auto e : pose_msg.pose.covariance){
		frobenius_norm += e*e;
	}

	frobenius_norm = std::sqrt(frobenius_norm);

	if(frobenius_norm < 0.005){
		localised_ = true;
		// unsubscribe from "amcl_pose" when localization finish
		amcl_pose_sub_.shutdown();
	}
}

void Stage2Node::mainLoop(){
	// wait for the Turtlebot to localise
	ROS_INFO("Localising...");
	while(ros::ok()){
		// turn slowly
		geometry_msgs::Twist twist{};
		twist.angular.z = 1.;
		cmd_vel_pub_.publish(twist);

		if(localised_){
			ROS_INFO("localised!");
			break;
		}

		ros::Duration(0.1).sleep();
	}

	// stop turning
	geometry_msgs::Twist twist{};
	twist.angular.z = 0;
	cmd_vel_pub_.publish(twist);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "stage2_node");

	ros::NodeHandle nh{};

	Stage2Node bs(nh);

	// Asynchronous spinner doesn't block
	ros::AsyncSpinner spinner(1);
	spinner.start();

	bs.mainLoop();
	return 0;
}