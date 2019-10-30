#include "brick_search/brick_search.h"

BrickSearch::BrickSearch(ros::NodeHandle& nh) : it_{nh}{
	// wait for "static map" service to be available
	ROS_INFO("Waiting for \"static_map\" service...");
	ros::service::waitForService("static_map");

	// get the map
	nav_msgs::GetMap get_map{};

	if(!ros::service::call("static_map", get_map)){
		ROS_ERROR("Unable to get map");
		ros::shutdown();
	}
	else{
		map_ = get_map.response.map;
		ROS_INFO("Map received");
	}

	// This allows you to access the map data as an Opencv image
	map_image_ = cv::Mat(map_.info.height, map_.info.width, CV_8U, &map_.data.front());

	// wait for the transform to become available
	ROS_INFO("Waiting for transform from \"map\" to \"base_link\"");
	while(ros::ok() && !transform_buffer_.canTransform("map", "base_link", ros::Time(0.))){
		ros::Duration(0.1).sleep();
	}
	ROS_INFO("Transform available!");

	// subscribe to "amcl_pose" to get pose covariance
	amcl_pose_sub_ = nh.subscribe("amcl_pose", 1, &BrickSearch::amclPoseCallback, this);

	// subscribe to the camera
	image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &BrickSearch::imageCallback, this);
	processed_image_pub_ = it_.advertise("/brick_search/processed_image", 1);

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

void BrickSearch::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg){
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

void BrickSearch::imageCallback(const sensor_msgs::ImageConstPtr& image_msg_ptr){
	if(image_msg_count_ < 5){
		image_msg_count_++;
		return;
	}

	image_msg_count_ = 0;
	
	// process image
	cv_bridge::CvImagePtr cv_ptr;
	// convert to cv image
    try
    {
    	cv_ptr = cv_bridge::toCvCopy(image_msg_ptr, sensor_msgs::image_encodings::BGR8);
    }
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat resizeImge, hsvImage, threshImage;
	cv::resize(~(cv_ptr->image), resizeImge, cv::Size(), 0.25, 0.25, cv::INTER_LINEAR);
	cv::cvtColor(resizeImge, hsvImage, cv::COLOR_BGR2HSV);
	
	cv::inRange(hsvImage, cv::Scalar(80, 70, 50), cv::Scalar(100, 255, 255), threshImage);

	cv::Moments m = cv::moments(threshImage, true);
	p.x = m.m10 / m.m00;
	p.y = m.m01 / m.m00;
	m00 = m.m00;

	cv::circle(threshImage, p, 10, cv::Scalar(0,0,0), -1);
	//cv::cvtColor(threshImage, outImage, cv::COLOR_HSV2BGR);
	cv::imshow( "Display window", threshImage);
	cv::waitKey(1);
	cv_bridge::CvImage out_msg;
	out_msg.header = image_msg_ptr->header;
	out_msg.image = threshImage;
	processed_image_pub_.publish(out_msg.toImageMsg());

	if(m00 > 200){
		brick_found_ = true;
	}
}

void BrickSearch::mainLoop(){
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

	while(ros::ok()){
		if(brick_found_){
			ROS_INFO("brick found!");
			break;
		}
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "break_search");

	ros::NodeHandle nh{};

	BrickSearch bs(nh);

	// Asynchronous spinner doesn't block
	ros::AsyncSpinner spinner(1);
	spinner.start();

	bs.mainLoop();
	return 0;
}