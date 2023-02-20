#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher cmd_vel_publisher;
geometry_msgs::Twist cmd_vel;
double current_x,current_y,current_ori_z;
float laser_distance;

void pose_callback(const nav_msgs::Odometry data)
{
  current_x = data.pose.pose.position.x;
  current_y = data.pose.pose.position.y;
  current_ori_z = data.pose.pose.orientation.z;
}

void laser_distance_callback(const std_msgs::Float32 msg)
{
  laser_distance = msg.data;
}

int rotation()
{
  //stop
  cmd_vel.linear.x=0;
  cmd_vel.linear.y=0;
  cmd_vel_publisher.publish(cmd_vel);

  ROS_INFO("rotate!");
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base turned right");
  else
    ROS_INFO("The base failed to turn right for some reason");

  return 0;
}

int go()
{
  ROS_INFO("go!");  
  cmd_vel.linear.x=0.1;
  cmd_vel.linear.y=0.1;
  cmd_vel_publisher.publish(cmd_vel);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel",100,true);
  ros::Subscriber pose_sub = n.subscribe("/odom",10, pose_callback);
  ros::Subscriber min_distance_sub = n.subscribe("/min_distance",10, laser_distance_callback);

  //goal.target_pose.header.frame_id = "map";

  while(ros::ok()){
    ROS_INFO("laser_distance: %f curreent_ori_z: %lf",laser_distance,current_ori_z);
    go();

    if(laser_distance < 35 && laser_distance > 0){
      rotation();
    }

    ros::spinOnce(); //callback 
  }
  return 0;
}
