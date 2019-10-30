#include "brick_search/stage3_node.h"

Stage3Node::Stage3Node(ros::NodeHandle& nh){
	// wait for the transform to become available
	ROS_INFO("Waiting for transform from \"map\" to \"base_link\"");
	while(ros::ok() && !transform_buffer_.canTransform("map", "base_link", ros::Time(0.))){
		ros::Duration(0.1).sleep();
	}
	ROS_INFO("Transform available!");

	// action client for "move_base"
	ROS_INFO("Waiting for \"move_base\" action...");
	move_base_action_client_.waitForServer();
	ROS_INFO("\"move_base\" action available!");

	// Advertise "cmd_vel" publisher to control TurtleBot manually
  	cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);

	// set up goal and image result callback
	goalID_sub_ = nh.subscribe("/explore_server/goal", 1, &Stage3Node::goalCallback, this);
	result_sub_ = nh.subscribe("/brick_search/image_result", 1, &Stage3Node::servoing, this);

	// set up exploration cancel publisher
	cancel_pub_ = nh.advertise<actionlib_msgs::GoalID>("/explore_server/cancel", 10);

	found_brick_ = false;
	approached_ = false;
}

void Stage3Node::servoing(brick_search::ImageResult msg){
	// if area of red larger than threshold, claim we find the brick!
	if(!found_brick_ && msg.m00 > 3000){
		ROS_INFO("find the brick!");
		cancel_pub_.publish(current_goalID);
		found_brick_ = true;
	}

	// if already found the brick but not close enough yet, move towards it
	if(found_brick_ && !approached_){
		geometry_msgs::Twist twist{};
		if(msg.m00 > 80000){ //480*270=129600
			approached_ = true;
			twist.linear.x = 0;
  			twist.angular.z = 0.;
  			ROS_INFO("finishing approaching!");
		}else{
			// image size is 480*270
			if(msg.center.x < 220)
				twist.angular.z = 0.2;
			else if(msg.center.x > 260)
				twist.angular.z = -0.2;
			else{
				twist.angular.z = 0;
				twist.linear.x = 0.2;
			}
		}
		cmd_vel_pub_.publish(twist);
	}

}

void Stage3Node::goalCallback(frontier_exploration::ExploreTaskActionGoal msg){
	ROS_INFO("receive a new goal: %s", msg.goal_id.id);
	current_goalID = msg.goal_id;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "stage3_node");
	ros::NodeHandle nh{};

	Stage3Node s3n(nh);

	ros::spin();

	return 0;
}