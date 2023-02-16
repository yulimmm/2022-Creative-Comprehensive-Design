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
double current_x,current_y,current_ori_z;

void pose_callback(const nav_msgs::Odometry data)
{
  current_x = data.pose.pose.position.x;
  current_y = data.pose.pose.position.y;
  current_ori_z = data.pose.pose.orientation.z;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel",100,true);
  ros::Subscriber pose_sub = n.subscribe("/odom",10, pose_callback);

  while(ros::ok()){
    ROS_INFO("%lf",current_ori_z);

    if(current_ori_z>-0.6 && current_ori_z<-0.1){
      ROS_INFO("right go");
      cmd_vel.linear.x=0;
      cmd_vel.linear.y=0;
      cmd_vel.angular.z=-0.2;
      cmd_vel_publisher.publish(cmd_vel);
    }

    if(current_ori_z<-0.7){
      cmd_vel.linear.x=0;
      cmd_vel.linear.y=0;
      cmd_vel.angular.z=-0;
      cmd_vel_publisher.publish(cmd_vel);
      ROS_INFO("stop");
    }
    
    ros::spinOnce();
  }
  return 0;
}
