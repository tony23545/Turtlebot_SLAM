#include "brick_search/stage3_plus_node.h"
// Constructor
Stage3PlusNode::Stage3PlusNode(ros::NodeHandle& nh)
{
  srand(time(0));

  // Wait for "static_map" service to be available
  ROS_INFO("Waiting for \"static_map\" service...");
  ros::service::waitForService("static_map");

  // Get the map
  nav_msgs::GetMap get_map{};

  if (!ros::service::call("static_map", get_map))
  {
    ROS_ERROR("Unable to get map");
    ros::shutdown();
  }
  else
  {
    map_ = get_map.response.map;
    ROS_INFO("Map received");
  }

  // This allows you to access the map data as an OpenCV image
  map_image_ = cv::Mat(map_.info.height, map_.info.width, CV_8U, &map_.data.front());
  cv::transpose(map_image_, map_image_);
  Point2f src_center(map_image_.cols/2.0F, map_image_.rows/2.0F);
  Mat rot_mat = getRotationMatrix2D(src_center, 90, 1.0);
  Mat dst;
  warpAffine(map_image_, map_image_, rot_mat, map_image_.size());

  //cv::imshow("ggg", map_image_);
  //cv::waitKey(0);

  // Wait for the transform to be become available
  ROS_INFO("Waiting for transform from \"map\" to \"base_link\"");
  while (ros::ok() && !transform_buffer_.canTransform("map", "base_link", ros::Time(0.)))
  {
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("Transform available");

  // Subscribe to "amcl_pose" to get pose covariance
  amcl_pose_sub_ = nh.subscribe("amcl_pose", 1, &Stage3PlusNode::amclPoseCallback, this);

  // Advertise "cmd_vel" publisher to control TurtleBot manually
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);

  // Action client for "move_base"
  ROS_INFO("Waiting for \"move_base\" action...");
  move_base_action_client_.waitForServer();
  ROS_INFO("\"move_base\" action available");
  move_base_status_sub_ = nh.subscribe("/move_base/result", 1, &Stage3PlusNode::exploring, this);

  // set up goal and image result callback
  goalID_sub_ = nh.subscribe("/move_base/goal", 1, &Stage3PlusNode::goalCallback, this);
  result_sub_ = nh.subscribe("/brick_search/image_result", 1, &Stage3PlusNode::servoing, this);

  // set up exploration cancel publisher
  cancel_pub_ = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);

  found_brick_ = false;
  approached_ = false;
  localised_ = false;

  reach_goal = 0;

  // Reinitialise AMCL
  ros::ServiceClient global_localization_service_client = nh.serviceClient<std_srvs::Empty>("global_localization");
  std_srvs::Empty srv{};
  global_localization_service_client.call(srv);

  processMap();
}

geometry_msgs::Point Stage3PlusNode::get_sample(int x, int y){
  cv::waitKey(0);
  int sampleX = x + 10;
  int sampleY = y + 10;
  cv::Mat part;
  part = map_image_(cv::Rect(sampleX - 4, sampleY - 4, 9, 9));
  cv::Moments m = cv::moments(part);

  while((map_image_.at<char>(sampleY, sampleX) != 0) || (m.m00 > 0)){
    sampleX = x + rand() % 20;
    sampleY = y + rand() % 20;
    part = map_image_(cv::Rect(sampleX - 4, sampleY - 4, 9, 9));
    m = cv::moments(part);
    ROS_INFO("%d, %d, %d, %f", sampleX, sampleY, map_image_.at<char>(sampleY, sampleX), m.m00);
  }
  geometry_msgs::Point p;
  p.x = sampleX;
  p.y = sampleY;
  ROS_INFO("x = %d, y = %d, xs = %d, ys = %d", x, y, sampleX, sampleY);
  return p;
}

void Stage3PlusNode::processMap(){
  ROS_INFO("processing map...");
  cv::Mat part;

  for(int y = 0; y < map_image_.rows; y+=20){
    if(y % 40 == 0){
      for(int x = 0; x < map_image_.cols; x+=20){
        part = map_image_(cv::Rect(x, y, 20, 20));
        cv::Moments m = cv::moments(part);
        if((m.m00 / 400) < 160){
          geometry_msgs::Pose p;
          p.position = get_sample(x, y);
          p.orientation.w = 1.0;
          explore_que.push(p);
        }
      }
    }
    else{
      for(int x = map_image_.cols-20; x >= 0; x-=20){
        part = map_image_(cv::Rect(x, y, 20, 20));
        cv::Moments m = cv::moments(part);
        if((m.m00 / 400) < 160){
          geometry_msgs::Pose p;
          p.position = get_sample(x, y);
          p.orientation.w = -1.0;
          explore_que.push(p);
        }
      }
    }
  }
}

void Stage3PlusNode::exploring(const move_base_msgs::MoveBaseActionResult& msg){
  if(localised_ && (!found_brick_)){
        geometry_msgs::Pose p = explore_que.front();
        explore_que.pop();
        p.position.x = (p.position.x -80) / 20;
        p.position.y = (80 - p.position.y) / 20;

        move_base_msgs::MoveBaseActionGoal action_goal{};

        action_goal.goal.target_pose.header.frame_id = "map";
        action_goal.goal.target_pose.pose = p;

        ROS_INFO("Sending goal (%f, %f)...", p.position.x, p.position.y);
        move_base_action_client_.sendGoal(action_goal.goal);
         
  }
}

void Stage3PlusNode::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg)
{
  // Check the covariance
  double frobenius_norm = 0.;

  for (const auto e : pose_msg.pose.covariance)
  {
    frobenius_norm += std::pow(e, 2.);
  }

  frobenius_norm = std::sqrt(frobenius_norm);

  if (frobenius_norm < 0.05)
  {
    geometry_msgs::Twist twist{};
    twist.angular.z = 0.;
    cmd_vel_pub_.publish(twist);

    // Unsubscribe from "amcl_pose" because we should only need to localise once at start up
    amcl_pose_sub_.shutdown();

    ros::Duration(1.0).sleep();
    geometry_msgs::Pose p = explore_que.front();
    explore_que.pop();
    p.position.x = (p.position.x -80) / 20;
    p.position.y = (80 - p.position.y) / 20;

    move_base_msgs::MoveBaseActionGoal action_goal{};

    action_goal.goal.target_pose.header.frame_id = "map";
    action_goal.goal.target_pose.pose = p;

    ROS_INFO("Sending goal (%f, %f) amcl...", p.position.x, p.position.y);
    move_base_action_client_.sendGoal(action_goal.goal);

    ros::Duration(1.0).sleep();
    localised_ = true;
  }
  else{
    geometry_msgs::Twist twist{};
    twist.angular.z = 1.;
    cmd_vel_pub_.publish(twist);
    }
}

void Stage3PlusNode::servoing(brick_search::ImageResult msg){
  // if area of red larger than threshold, claim we find the brick!
  if(!found_brick_ && msg.m00 > 3000){
    ROS_INFO("find the brick!");
    cancel_pub_.publish(current_goalID);
    found_brick_ = true;
    localised_ = true;
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

void Stage3PlusNode::goalCallback(move_base_msgs::MoveBaseActionGoal msg){
  ROS_INFO("receive a new goal: %s", msg.goal_id.id);
  current_goalID = msg.goal_id;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stage3_plus_node");

  ros::NodeHandle nh{};

  Stage3PlusNode s3pn(nh);

  ros::spin();

  return 0;
}
