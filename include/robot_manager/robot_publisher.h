#ifndef ROBOT_PUBLISHER_INCLUDED
#define ROBOT_PUBLISHER_INCLUDED

#include "ford_msgs/Clusters.h"
#include "ford_msgs/PlannerMode.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Header.h"
#include "tf/transform_datatypes.h"
#include "robot_manager/Robot_Task_Request.h"
#include "warehouse_manager/RobotInfo.h"
#include "std_srvs/Empty.h"
#include <std_msgs/Int32.h>

#include <string>

class Robot {
private:
  /**
  struct obstacles
  {
    std_msgs::Header header;
    uint32_t label;
    geometry_msgs::Point mean_point;
    geometry_msgs::Point min_point;
    geometry_msgs::Point max_point;
    geometry_msgs::Vector3 velocity;
    float orientation;
  };
  **/
  
  ros::Subscriber position;
  ros::Subscriber camera_info;
  ros::Subscriber depth;
  ros::Subscriber temp_sub; // This is temp for robot 1, it has to be
                            // generalised
  ros::NodeHandle nh;
  std::string name;

public:
  float dist;
  double begin;
  double end;
  float dist_to_goal;
  float dist_to_goal_prev;
  int counter_cancel;
  bool robot_is_busy;
  sensor_msgs::PointCloud2 cloud;
  ros::Publisher pose;
  ros::Publisher velocity;
  ros::Publisher goal;
  ros::Publisher cluster;
  ros::Publisher mode;
  ros::Subscriber global_planning;
  ros::Publisher intiPose; 
  ros::Publisher goalPose; 
  float yaw;
  //EnvironmentMaster env;
  std::vector<Robot> all_robots; 
  std_msgs::Header head;
  geometry_msgs::PoseStamped pose_robot;
  geometry_msgs::Vector3 velocity_robot;
  geometry_msgs::PoseStamped goal_robot;
  geometry_msgs::PoseStamped local_goal_robot;
  sensor_msgs::CameraInfo info; // Later this can be subscribed to only once
  sensor_msgs::Image depth_image;
  ros::ServiceClient request_goal;
  nav_msgs::Path planned_path;
  ford_msgs::Clusters clust;
  /**
   * @brief This method is called under ros infinite loop for all robots,
   *        this method assignes subgoals to local planner and which the task is finished it
   *        requests for new task  
   * @param input void
   * @param output void
   */
  void runRobot();
  /**
   * @brief Empty constructor
   */
  Robot() { std::cout << "Robot added" << std::endl;};
  /**
   * @brief Initialize the class object woth publishers and subscribers
   * @param input nh It is the node handle for that particular robot ros node
   * @param input s It is the string of Robot name
   * @param output void
   */
  void init(ros::NodeHandle nh, std::string s);
  /**
   * @brief Destroy the Robot object
   * 
   */
  ~Robot() { std::cout << "Robot removed" << std::endl; };
  /**
   * @brief Callback method for robot position subscriber
   * @param input msg It is the robot position information published by stage
   * @param output void
   */
  void positionCB(const nav_msgs::Odometry::ConstPtr &msg);
  /**
   * @brief Callback method for robot camera sensor subscriber
   * @param input msg It is the camera parameters data published by stage
   * @param output void
   */
  void cameraInfoCB(const sensor_msgs::CameraInfo::ConstPtr &msg);
  /**
   * @brief Callback method for robot camera sensor depth image subscriber
   * @param input msg It is the depth image published by stage
   * @param output void
   */
  void depthCB(const sensor_msgs::Image::ConstPtr &msg);
  /**
   * @brief Temp point to subscribe to points
   * @param input void
   * @param output void
   */
  void robot1_pub();
  /**
   * @brief Temp point cloud callback
   * @param input msg It is the point cloud data
   * @param output void
   */
  void fordCB(const sensor_msgs::PointCloud2::ConstPtr &msg);
  /**
   * @brief This method is used to get the netowrk input information fro mother robot objects
   * @param input r robot object
   * @param output ford_msgs::Clusters 
   */
  ford_msgs::Clusters getMessage(Robot r);
  
  /**
   * @brief Set the planner mode to 1 and return planner mode data dtype
   * @param input void
   * @param output 
   */
  ford_msgs::PlannerMode getMode();
  /**
   * @brief Callback for global planner publisher
   * @param input msg List of waypoints to final goal
   * @param output void
   */
  void globalPlanningCB(const nav_msgs::Path::ConstPtr &msg);
  /**
   * @brief Method to get neighbour robots
   * @param input void
   * @param output ford_msgs::Clusters Input to the network
   */
  ford_msgs::Clusters getInRange();
  
};

#endif