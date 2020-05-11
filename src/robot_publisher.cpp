#include <robot_manager/robot_publisher.h>
#include<unordered_map>
#include <tuple>
#include <fstream>
struct CompareNodelet
{
    bool operator()(nav_msgs::Odometry const& lhs, nav_msgs::Odometry const& rhs) const;
};

void Robot::init(ros::NodeHandle nh, std::string s) {
  this->name = s;
  
  this->nh = nh;
  this->position = this->nh.subscribe("/" + s + "/base_pose_ground_truth", 1,
                                      &Robot::positionCB, this);
  this->camera_info = this->nh.subscribe("/" + s + "/camera_info", 1,
                                         &Robot::cameraInfoCB, this);
  this->depth =
      this->nh.subscribe("/" + s + "/depth", 1, &Robot::depthCB, this);
  std::string temp;
  
  if(s.size() == 9){
    
    temp = "robot";
    std::string temp2;
    temp2 = s[s.size()-1];
    temp.append(temp2);
    
  }
  else{
   
    temp = "robot";
    std::string temp2;
    temp2 = s[s.size()-2];
    temp.append(temp2);
    std::string temp3;
    temp3 = s[s.size()-1];
    temp.append(temp3);
   
  }
  
  pose =
      this->nh.advertise<geometry_msgs::PoseStamped>("/" + temp + "/pose", 10);
  velocity =
      this->nh.advertise<geometry_msgs::Vector3>("/" + temp + "/velocity", 10);
  goal = this->nh.advertise<geometry_msgs::PoseStamped>(
      "/" + temp + "/move_base_simple/goal", 10);
  cluster = this->nh.advertise<ford_msgs::Clusters>(
      "/" + temp + "/cluster/output/clusters", 10);
  mode = this->nh.advertise<ford_msgs::PlannerMode>(
      "/" + temp + "/cadrl_node/mode", 10);
  
  global_planning = this->nh.subscribe(s + "/nav_path", 1,
                                         &Robot::globalPlanningCB, this);

  request_goal = this->nh.serviceClient<robot_manager::Robot_Task_Request>("/request_available_task");

  intiPose = this->nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(s + "/initialpose", 10);
  goalPose = this->nh.advertise<geometry_msgs::PoseStamped>(s + "/move_base_simple/findgoal", 10);
  std::string sq = s + "/move_base_simple/findgoal";
  
  robot_is_busy = false;
  counter_cancel = 0;
  dist_to_goal = 0;
  dist_to_goal_prev = 0;
    
}

void Robot::runRobot(){
  if(robot_is_busy){
      
      if(planned_path.poses.size() > 0){
        counter_cancel = 0;
        ROS_INFO_STREAM(this->name << " Robot at work");
        if(abs(pose_robot.pose.position.x - planned_path.poses[0].pose.position.x) <= 0.03 && abs(pose_robot.pose.position.y - planned_path.poses[0].pose.position.y) <= 0.03){
          while(abs(pose_robot.pose.position.x - planned_path.poses[0].pose.position.x) <= 6 && abs(pose_robot.pose.position.y - planned_path.poses[0].pose.position.y) <= 6 && planned_path.poses.size() != 1){
          planned_path.poses.erase(planned_path.poses.begin());
          //New Local Goal
          local_goal_robot = planned_path.poses[0]; 
          }
          if(planned_path.poses.size() == 1){
            local_goal_robot = planned_path.poses[0];
          }
        }
        else{
          //Always goal at position 0
          local_goal_robot = planned_path.poses[0];
        }
        pose.publish(pose_robot);
        velocity.publish(velocity_robot);
        goal.publish(local_goal_robot);
        
        cluster.publish(getInRange()); // Publish the robots in the environment which are neighbour to the current robot
        
        mode.publish(getMode());
      }
      else{
        counter_cancel ++;
        if(counter_cancel > 10){
          robot_is_busy = false;
        }
      }
      if(abs(pose_robot.pose.position.x - goal_robot.pose.position.x) <= 0.3 && abs(goal_robot.pose.position.y - goal_robot.pose.position.y) <= 0.3){
        robot_is_busy = false;
        end = ros::Time::now().toSec();
        double time_taken = end - begin;
        ROS_INFO_STREAM(this->name << " Goal Found in: " << time_taken << " sec and distance travelled: " << dist );
        std::ofstream outfile;
        outfile.open("/home/krishna/cadrl_ws/src/timesheet.txt", std::ios_base::app); // append instead of overwrite
        outfile << this->name << " " << time_taken << " " << dist  << " " << goal_robot.pose.position.x << ", " << goal_robot.pose.position.y << "\n" ;
        outfile.close();
        dist = 0;
        
      }

  }
  else{
      //Request for goal
      robot_manager::Robot_Task_Request srv;
      std::string robot_number;
      if(this->name.size() == 9) robot_number = this->name[this->name.size() - 1];
      else{
        robot_number = this->name[this->name.size() - 2];
        std::string temp;
        temp = this->name[this->name.size() - 1];
        robot_number.append(temp);
      } 
      srv.request.name = robot_number;
      ros::Duration t(5.0);
      ROS_INFO_STREAM("Get Goal");
      if(request_goal.call(srv)){
        ROS_INFO_STREAM(this->name << " is requesting for goal");
        std::vector<float> g;
        g.push_back(srv.response.x);
        g.push_back(srv.response.y);
        //g = env.getMeGoal();
        if(g.size() > 0){
          //Set Goal Pose
          goal_robot.header = pose_robot.header;
          goal_robot.pose = pose_robot.pose;
          goal_robot.pose.position.x = g[0];
          goal_robot.pose.position.y = g[1];
          ROS_INFO_STREAM(this->name << "Start is = (" << pose_robot.pose.position.x << ", " << pose_robot.pose.position.y << ")  ;  Goal recieved is = (" << goal_robot.pose.position.x << ", " << goal_robot.pose.position.y << ")");
          //Find the path using Astar
          dist = (pose_robot.pose.position.x - goal_robot.pose.position.x)*(pose_robot.pose.position.x - goal_robot.pose.position.x) + (pose_robot.pose.position.y - goal_robot.pose.position.y)*(pose_robot.pose.position.y - goal_robot.pose.position.y);
        }
        else{
        }
        begin = ros::Time::now().toSec();
        robot_is_busy = true;
      }
      else{
        ROS_INFO_STREAM("Expired");
      }

  }
  float local_to_current = (local_goal_robot.pose.position.x - pose_robot.pose.position.x)*(local_goal_robot.pose.position.x - pose_robot.pose.position.x) + (local_goal_robot.pose.position.y - pose_robot.pose.position.y)*(local_goal_robot.pose.position.y - pose_robot.pose.position.y);
  if(planned_path.poses.size() == 0 ){
        ROS_INFO_STREAM(this->name << "is requesting path" << " " << goal_robot.pose.position.x << ", " << goal_robot.pose.position.y);
        geometry_msgs::PoseWithCovarianceStamped iniy;
        iniy.header = pose_robot.header;
        iniy.pose.pose.position = pose_robot.pose.position;
        intiPose.publish(iniy);
        goalPose.publish(goal_robot);
  }
  if(!robot_is_busy) planned_path.poses.clear();

}


void Robot::globalPlanningCB(const nav_msgs::Path::ConstPtr &msg){
  if(msg->poses.size() > 0){
    ROS_INFO_STREAM(this->name <<" Path found!!");
    planned_path.header = msg->header;
    planned_path.poses = msg->poses;
  }
}

void Robot::positionCB(const nav_msgs::Odometry::ConstPtr &msg) {
  pose_robot.header = msg->header;
  head = msg->header;
  pose_robot.pose = msg->pose.pose;
  velocity_robot.x = msg->twist.twist.linear.x;
  velocity_robot.y = msg->twist.twist.linear.y;
  velocity_robot.z = msg->twist.twist.linear.z;
  tf::Pose pose;
  tf::poseMsgToTF(msg->pose.pose, pose);
  yaw = tf::getYaw(pose.getRotation());
}

void Robot::cameraInfoCB(const sensor_msgs::CameraInfo::ConstPtr &msg) {
  info = *msg;
}

void Robot::depthCB(const sensor_msgs::Image::ConstPtr &msg) {
  depth_image = *msg;
}

void Robot::robot1_pub() {
  this->temp_sub = this->nh.subscribe("/" + this->name + "/points", 10,
                                      &Robot::fordCB, this);
}

void Robot::fordCB(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  cloud = *msg;
}

ford_msgs::Clusters Robot::getMessage(Robot r) {
  ford_msgs::Clusters msg;
  msg.header = pose_robot.header;
  msg.labels.push_back(1);
  //msg.counts.push_back(1);
  geometry_msgs::Point mean, minp, maxp;
  mean = pose_robot.pose.position;
  msg.mean_points.push_back(mean);
  minp.x = mean.x - 0.05;
  minp.y = mean.y - 0.05;
  minp.z = mean.z;
  msg.max_points.push_back(minp);
  maxp.x = mean.x + 0.05;
  maxp.y = mean.y + 0.05;
  maxp.z = mean.z;
  msg.min_points.push_back(maxp);
  //msg.pointclouds.push_back(r.cloud);
  msg.velocities.push_back(velocity_robot);
  msg.orientation.push_back(yaw);
  return msg;
}




ford_msgs::PlannerMode Robot::getMode() {
  ford_msgs::PlannerMode msg;
  msg.header = pose_robot.header;
  msg.mode = 1;
  return msg;
}



ford_msgs::Clusters Robot::getInRange(){
  ford_msgs::Clusters msg;
  int count = 0;
  boost::shared_ptr<warehouse_manager::RobotInfo const> point;
  warehouse_manager::RobotInfo pos;
  point = ros::topic::waitForMessage<warehouse_manager::RobotInfo>("/robots_information",this->nh);
  if(point != NULL){
    pos = *point;
  }

  //std::map<nav_msgs::Odometry, float, CompareNodelet> yaww;
  //std::map<nav_msgs::Odometry, int, CompareNodelet> labell;
  std::map<std::tuple<float, float>, float> yaww;
  std::map<std::tuple<float, float>, int> labell;
  //This can be imporved by sorting only neighbours
  for(int j = 0; j < pos.positions.size(); j++){
    auto t = std::make_tuple(pos.positions[j].pose.pose.position.x, pos.positions[j].pose.pose.position.y);
    yaww.insert({t, pos.yaws[j]});
    labell.insert({t, pos.labels[j]});
  }
  int xqq = pose_robot.pose.position.x;
  int yqq = pose_robot.pose.position.y;
  std::sort(pos.positions.begin(), pos.positions.end(), [&xqq, &yqq](nav_msgs::Odometry a, nav_msgs::Odometry b){return (((a.pose.pose.position.x - xqq)*(a.pose.pose.position.x - xqq)) + ((a.pose.pose.position.y - yqq)*(a.pose.pose.position.y - yqq))) < (((b.pose.pose.position.x - xqq)*(b.pose.pose.position.x - xqq)) + ((b.pose.pose.position.y - yqq)*(b.pose.pose.position.y - yqq)));});
  //std::sort(pos.positions.begin(), pos.positions.end(), [&xqq, &yqq](nav_msgs::Odometry a, nav_msgs::Odometry b){return false;});
  for(int k = 0; k < pos.positions.size(); k++){
    auto t = std::make_tuple(pos.positions[k].pose.pose.position.x, pos.positions[k].pose.pose.position.y);
    pos.yaws[k] = yaww[t];
    pos.labels[k] = labell[t];
  }

  for(int i = 0; i < pos.positions.size(); i++){
    geometry_msgs::Point mean, minp, maxp;
    mean.x = pos.positions[i].pose.pose.position.x;
    mean.y = pos.positions[i].pose.pose.position.y;
    mean.z = pos.positions[i].pose.pose.position.z;
    if(pose_robot.pose.position.x == mean.x && pose_robot.pose.position.y == mean.y){
      continue;
    }
    else{
      //if(((cos(yaw+1.309)*(mean.y - pose_robot.pose.position.y) - sin(yaw+1.309)*(mean.x - pose_robot.pose.position.x)) <=0) &&  ((cos(yaw-1.309)*(mean.y - pose_robot.pose.position.y) - sin(yaw-1.309)*(mean.x - pose_robot.pose.position.x)) >=0) && (((mean.x - pose_robot.pose.position.x) * (mean.x - pose_robot.pose.position.x) + (mean.y - pose_robot.pose.position.y)*(mean.y - pose_robot.pose.position.y)) <= 12.25)){
        if(((mean.x - pose_robot.pose.position.x)*(mean.x - pose_robot.pose.position.x)) + ((mean.y - pose_robot.pose.position.y)*(mean.y - pose_robot.pose.position.y)) <= 12.25){
        msg.mean_points.push_back(mean);
        msg.header = pos.positions[i].header;
        msg.labels.push_back(pos.labels[i]);
        
        minp.x = mean.x - 0.25;
        minp.y = mean.y - 0.25;
        minp.z = mean.z;
        msg.max_points.push_back(minp);
        maxp.x = mean.x + 0.25;
        maxp.y = mean.y + 0.25;
        maxp.z = mean.z;
        msg.min_points.push_back(maxp);

        geometry_msgs::Vector3 vel;
        vel = pos.positions[i].twist.twist.linear;
        msg.velocities.push_back(vel);
        msg.orientation.push_back(pos.yaws[i]);
        count ++;
      }
      else{
        //ROS_WARN_STREAM(this->name << " " << count);
        return msg;
      }
    }
    
  }
  
  return msg;
}