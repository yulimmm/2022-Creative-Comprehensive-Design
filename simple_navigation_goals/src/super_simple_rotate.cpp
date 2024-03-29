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

ros::Publisher cmd_vel_publisher;
geometry_msgs::Twist cmd_vel;
double current_x,current_y,current_ori_z,current_ori_w;
float laser_distance;

void pose_callback(const nav_msgs::Odometry data)
{
  current_x = data.pose.pose.position.x;
  current_y = data.pose.pose.position.y;
  current_ori_z = data.pose.pose.orientation.z;
  current_ori_w = data.pose.pose.orientation.w;
  //ROS_INFO("x:%lf y:%lf z:%lf",current_x,current_y,current_ori_z);
}

void laser_distance_callback(const std_msgs::Float32 msg)
{
  laser_distance = msg.data;
}

int rotation()
{
  ROS_INFO("rotate!");
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.linear.z = 0;
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = 1;

  ros::Rate rate(10);
  double time = 0;
  while(time<1.57)
  {
     cmd_vel_publisher.publish(cmd_vel);
     time += 0.1;
     rate.sleep();
  }
  return 0;
}

int go()
{
  ROS_INFO("go!");  
  cmd_vel.linear.x = 0.1;
  cmd_vel.linear.y = 0.1;
  cmd_vel.linear.z = 0;
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = 0;
  cmd_vel_publisher.publish(cmd_vel);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel",100,true);
  ros::Subscriber pose_sub = n.subscribe("/t265/odom/sample",10, pose_callback);
  ros::Subscriber min_distance_sub = n.subscribe("/min_distance",10, laser_distance_callback);

  while(ros::ok()){
    ROS_INFO("laser_distance: %f curreent_ori_z: %lf",laser_distance,current_ori_z);
    go();

    if(laser_distance>0 && laser_distance < 40){
      rotation();
    }

    ros::spinOnce(); //callback 
  }
  return 0;
}
