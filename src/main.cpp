#include "ros/ros.h"
#include <iostream>
#include <robot_manager/robot_publisher.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

//#include <tf/transform_listener.h>
//#include <costmap_2d/costmap_2d_ros.h>
//#include <navfn/navfn_ros.h>







int main(int argv, char **argc) {
  ros::init(argv, argc, "robot_master");
  ros::NodeHandle nh_robot;
  ros::Rate loop_rate(10);
  std::string ns = ros::this_node::getNamespace();
  /**
  tf::TransformListener tf(ros::Duration(10));
  costmap_2d::Costmap2DROS costmap("my_costmap", tf);

  navfn::NavfnROS navfn;
  navfn.initialize("my_navfn_planner", &costmap);
  
  boost::shared_ptr<nav_msgs::OccupancyGrid const> shared_map;
  nav_msgs::OccupancyGrid map;
  shared_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map",nh);
  if(shared_map != NULL){
    map = *shared_map;
  }

  costmap.updateMap();

  **/



  //ns = "robot_0"; 
  //EnvironmentMaster env(nh_env);
  //Robot r1;
  Robot r0;
  //env.init("/home/krishna/cadrl_ws/src/warehouse_manager/params/location.txt");
  //env.init(3);
  r0.init(nh_robot, ns);
  //r1.init(nh_robot, "robot_1");
  //std::vector<Robot> v{r0, r1};
  //r0.all_robots = v;
  //r1.all_robots = v;
  //std::vector<Robot> robots{r0, r1};
  /**
  std::vector<geometry_msgs::PoseStamped> plan;
  navfn.makePlan(goal_0, r0.pose_robot, plan);
  for(auto a: plan){
    ROS_INFO_STREAM(a.pose.position.x << ", " << a.pose.position.y);
  }
  **/
  ros::Duration(2.5).sleep();
  while(ros::ok()){
    r0.runRobot();
    //r1.runRobot();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

 /** 
  while (ros::ok()) {
    //env.assign_task();
    //std::vector<std::string> robot_list;
    //std::vector<std::string> task_list;
    //for (auto a : env.robot_task_pair) {
    //  robot_list.push_back(a.first);
    //  task_list.push_back(a.second);
    //}
    //warehouse_manager::TaskInfo msg;
    //msg.robot_list = robot_list;
    //msg.task_list = task_list;
    //env.task_assignment.publish(msg);
    //iniy.header = r0.pose_robot.header;
    //iniy.pose.pose.position = r0.pose_robot.pose.position;
    /**
    r1.pose.publish(r1.pose_robot);
    r1.velocity.publish(r1.velocity_robot);
    r1.goal.publish(goal_1);
    r1.cluster.publish(r0.getMessage(r1));
    r1.mode.publish(r1.getMode());
    
    r0.pose.publish(r0.pose_robot);
    r0.velocity.publish(r0.velocity_robot);
    r0.goal.publish(goal_0);
    ford_msgs::Clusters fm;
    //r0.cluster.publish(getStatic(r1.head, r1.getMessage(r0)));
    //r0.cluster.publish();
    r0.mode.publish(r0.getMode());
    
    //intiPose.publish(iniy);
    //goalPose.publish(goal_0);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
**/