/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Vector3.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

geometry_msgs::PoseStamped pose;
mavros_msgs::State current_state;
bool start = 0;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void chatterCallback(geometry_msgs::Vector3 msg)
{
  if (start){
  if(msg.x < 0){
    msg.x = 360.0 + msg.x;
  }
  msg.x = (int)(10 * ((msg.x-220)/40));msg.x = msg.x/10;
  msg.y = (int)(10 * (msg.y)/(-40));msg.y = msg.y/10;msg.y = msg.y+2;
  msg.z = (int)(10 * (msg.z)/(-45));msg.z = msg.z/10;
  ROS_INFO("I heard X : [%f]",msg.x);
  ROS_INFO("I heard Y : [%f]",msg.y);
  ROS_INFO("I heard Z : [%f]",msg.z);
  pose.pose.position.x = -msg.z;
  pose.pose.position.y = msg.x;
  pose.pose.position.z = msg.y;
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber sub = nh.subscribe("/myo_raw/myo_ori_deg", 1000, chatterCallback);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    bool set=false;
    bool startup=false;
    float pi = 3.1415;
    ros::Time last_request = ros::Time::now();
    float theta=0;
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    ROS_INFO("Startup enabled");
                    start=true;
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
