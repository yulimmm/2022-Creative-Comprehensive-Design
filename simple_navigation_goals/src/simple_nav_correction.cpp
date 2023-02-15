#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher cmd_vel_publisher;

void turn_right();
void callback(const std_msgs::Float32 msg);
void turn_straight();
void turn_down();
void turn_left();
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped data);

double current_x;
double current_y;
double current_or_z;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  geometry_msgs::Twist cmd_vel;
  cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel",100,true);
  ros::Subscriber pose_sub = n.subscribe("/amcl_pose",10, pose_callback);
  //ros::Subscriber min_distance_sub = n.subscribe("/min_distance",10, callback);

  //ros::spin();
  turn_straight();  //0 1
  turn_right(); //-0.7 0.7
  turn_down();  //-1 0
  turn_left();  //0.7 0.7

  return 0;
}

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped data)
{
  current_x = data.pose.pose.position.x;
  current_y = data.pose.pose.position.y;
  current_or_z = data.pose.pose.orientation.z;
}


void turn_right()
{
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();


  goal.target_pose.pose.position.x = current_x; 
  goal.target_pose.pose.position.y = current_y;
  
  goal.target_pose.pose.orientation.x = 0; 
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = -0.7; 
  goal.target_pose.pose.orientation.w = 0.7; 

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base turned right");
  else
    ROS_INFO("The base failed to turn left for some reason");
}

void turn_straight()
{
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = current_x; 
  goal.target_pose.pose.position.y = current_y;

  goal.target_pose.pose.orientation.x = 0; 
  goal.target_pose.pose.orientation.y = 0; 
  goal.target_pose.pose.orientation.z = 0; 
  goal.target_pose.pose.orientation.w = 1; 

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base turned straight");
  else
    ROS_INFO("The base failed to turn left for some reason");
}

void turn_down()
{
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = current_x; 
  goal.target_pose.pose.position.y = current_y;

  goal.target_pose.pose.orientation.x = 0; 
  goal.target_pose.pose.orientation.y = 0; 
  goal.target_pose.pose.orientation.z = -1; 
  goal.target_pose.pose.orientation.w = 0; 

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base turned down");
  else
    ROS_INFO("The base failed to turn left for some reason");
}

void turn_left()
{
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = current_x; 
  goal.target_pose.pose.position.y = current_y;

  goal.target_pose.pose.orientation.x = 0; 
  goal.target_pose.pose.orientation.y = 0; 
  goal.target_pose.pose.orientation.z = 0.7; 
  goal.target_pose.pose.orientation.w = 0.7; 

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base turned left");
  else
    ROS_INFO("The base failed to turn left for some reason");
}

void callback(const std_msgs::Float32 msg)
{
  geometry_msgs::Twist cmd_vel;
//  ROS_INFO("go! result = %f\n",msg.data);

  if(msg.data>30){
    ROS_INFO("go! distance = %f\n",msg.data);

    cmd_vel.linear.x=0.1;
    cmd_vel.linear.y=0.1;
    cmd_vel_publisher.publish(cmd_vel);

  }
  else{
    ROS_INFO("stop! distance = %f",msg.data);
    cmd_vel.linear.x=0;
    cmd_vel.linear.y=0;
    cmd_vel_publisher.publish(cmd_vel);
    turn_right();
  }

}

void correction(double current_ori_z)
{
  if(current_ori_Z<0.35&&current_ori_z>-0.35){
    
  }
}
