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
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>


#include <brick_search/ImageResult.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <queue>

#include <ctime>
using namespace cv;
using namespace std;

class Stage3PlusNode
{
public:
  // Constructor
  explicit Stage3PlusNode(ros::NodeHandle& nh);

  // Publich methods
  void mainLoop();

private:
  // Variables
  nav_msgs::OccupancyGrid map_{};
  cv::Mat map_image_{};
  std::queue<geometry_msgs::Pose> explore_que;
  std::atomic<bool> localised_{ false };
  std::atomic<bool> brick_found_{ false };

  // Transform listener
  tf2_ros::Buffer transform_buffer_{};
  tf2_ros::TransformListener transform_listener_{ transform_buffer_ };

  // Subscribe to the AMCL pose to get covariance
  ros::Subscriber amcl_pose_sub_{};

  // Velocity command publisher
  ros::Publisher cmd_vel_pub_{};

  // Action client
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_client_{ "move_base", true };
  ros::Subscriber move_base_status_sub_{};

  // goal and image result subscriber
  ros::Subscriber goalID_sub_{};
  ros::Subscriber result_sub_{};

  // store current goal ID to cancel exploration when needed
  actionlib_msgs::GoalID current_goalID;

  // exploration cancel publisher
  ros::Publisher cancel_pub_{};

  bool found_brick_, approached_;
  bool send_goal;
  int reach_goal;

  int upleft_x, upleft_y, botright_x, botright_y;

  // Private methods
  geometry_msgs::Pose2D getPose2d();
  void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg);

  void servoing(brick_search::ImageResult msg);
  void goalCallback(move_base_msgs::MoveBaseActionGoal msg);
  
  geometry_msgs::Point get_sample(int x, int y);
  void processMap();

  void exploring(const move_base_msgs::MoveBaseActionResult& msg);
};