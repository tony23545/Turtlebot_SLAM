#include "brick_search/stage4_node.h"

Stage4Node::Stage4Node(ros::NodeHandle& nh) : it_{nh}{
	ROS_INFO("Setting up image node for brick search...");
	// subscribe to the camera
	image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &Stage4Node::imageCallback, this);
	processed_image_pub_ = it_.advertise("/brick_search/processed_image", 10);

	result_pub_ = nh.advertise<brick_search::ImageResult>("/brick_search/image_result", 10);
}

void Stage4Node::imageCallback(const sensor_msgs::ImageConstPtr& image_msg_ptr){
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
	cv::Point p(m.m10/m.m00, m.m01/m.m00);
	cv::circle(threshImage, p, 10, cv::Scalar(0,0,0), -1);
	//cv::cvtColor(threshImage, outImage, cv::COLOR_HSV2BGR);
	cv::imshow( "Display window", threshImage);
	cv::waitKey(1);

	// publish processed image
	cv_bridge::CvImage out_msg;
	out_msg.header = image_msg_ptr->header;
	out_msg.image = threshImage;
	processed_image_pub_.publish(out_msg.toImageMsg());

	// 
	brick_search::ImageResult result;
	result.m00 = m.m00;
	geometry_msgs::Point center;
	center.x = p.x;
	center.y = p.y;
	result.center = center;
	result_pub_.publish(result);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "stage4_node");

	ros::NodeHandle nh{};

	Stage4Node s4n(nh);

	// Asynchronous spinner doesn't block
	ros::spin();

	return 0;


}