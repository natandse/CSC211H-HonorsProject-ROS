
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include<std_msgs/String.h>

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

ros::Publisher vel_pub;

void request_shutdown(int signal)
{
  g_request_shutdown = 1;
}

void vel_publish(ros::Publisher& vel_pub,
                 geometry_msgs::Twist vel_msg,
                 double pub_secs)
{
  ros::WallTime start_time = ros::WallTime::now();
  ros::WallDuration pub_duration = ros::WallDuration(pub_secs);
  ros::WallTime end_time = start_time + pub_duration;

  ROS_INFO_STREAM("## Publishing velocity. ##");
  while (ros::WallTime::now() < end_time)
  {
    vel_pub.publish(vel_msg);

    ros::WallDuration(0.1).sleep();
  }

  ROS_INFO_STREAM("## Stopping. ##");
  geometry_msgs::Twist stop_msg;
  vel_pub.publish(stop_msg);
}

void move_linear(ros::Publisher& vel_pub, double vel_x, double pub_secs)
{
  geometry_msgs::Twist forward_msg;
  forward_msg.linear.x = vel_x;

  vel_publish(vel_pub, forward_msg, pub_secs);
}

void turn(ros::Publisher& vel_pub, double vel_z, double pub_secs)
{
  geometry_msgs::Twist right_turn_msg;
  right_turn_msg.angular.z = vel_z;

  vel_publish(vel_pub, right_turn_msg, pub_secs);
}

void moveCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  if (msg->data == "forward")
  {
    ROS_INFO("Moving forward.");
    move_linear(vel_pub, 0.25, 1.0);
  }
  else if(msg->data == "backward")
  {
    ROS_INFO("Moving backward.");
    move_linear(vel_pub, -0.25, 1.0);
  }
  else if(msg->data == "right")
  {
    ROS_INFO("Turning right.");
    turn(vel_pub, -1.6, 1.0);
  }
  else if(msg->data == "left")
  {
    ROS_INFO("Turning left.");
    turn(vel_pub, 1.6, 1.0);
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "move_on_command");

  ros::NodeHandle n;
  signal(SIGINT, request_shutdown);

  ros::Rate loop_rate(10);

  vel_pub= n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber sub = n.subscribe("/move_command", 1000, moveCallback);

  ROS_INFO_STREAM("## Listening for commands on /move_command ##");
  while (! g_request_shutdown)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO_STREAM("## Shutting down. ##");
  geometry_msgs::Twist stop_msg;
  vel_publish(vel_pub, stop_msg, 1.0);
  ros::shutdown();

  return 0;
}
