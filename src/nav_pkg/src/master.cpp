#include <iostream>
#include "ros/ros.h"
#include <cmath>
#include <string>
#include <vector>

#include "extern_variables.h"
#include "scout_msgs/scout_status.h" // �´�...?
#include "scout_msgs/cmd_vel.h" // �´�...?

using namespace std;

Local local;
Control control;
bool egoTopicFlag = false;

void egoTopicCallback(const scout_msgs::scout_status::ConstPtr& msg) {
    cout << "Ego topic callback is working" << endl;
    egoTopicFlag = true;
    ext_ego_cur_linear_velocity = msg->linear_velocity;
    ext_ego_cur_angular_velocity = msg->angular_velocity;

    cout << "linear velocity : "<< ext_ego_cur_linear_velocity << endl;
    cout << "angular velocity : "<< ext_ego_cur_angular_velocity << endl;
}

// TO DO
// LiDAR Localization : callback function, data -> ext 변수에 저장, localization 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "master_node");
    ros::NodeHandle nh;

    scout_msgs::cmd_vel ctrl_cmd_msg;
    ros::Publisher ctrl_cmd_pub = nh.advertise<scout_msgs::cmd_vel>("/cmd_vel", 10);
    //ros::Subscriber gps_sub = nh.subscribe("/gps", 100, gpsCallback);
    //ros::Subscriber imu_sub = nh.subscribe("/imu", 100, imuCallback);
    //ros::Subscriber lidar_sub = nh.subscribe("/detection3d_result", 10, LiDARCallback);
    ros::Subscriber ego_topic_sub = nh.subscribe("/scout_status", 10, egoTopicCallback);

    local.readCSV();

    while (ros::ok())
    {
        ctrl_cmd_msg.linear.x = ext_linear_velocity_input;
        ctrl_cmd_msg.angular.z = ext_angular_velocity_input;
        ctrl_cmd_pub.publish(ctrl_cmd_msg);
        egoTopicFlag = false;

        // cout << "target_velocity : " << ext_linear_velocity_input << endl;
        // cout << "ego velocity : " << ext_ego_velocity << endl;
        // cout << "State : " << ext_ego_state << "  Behavior : " << ext_ego_behavior << endl;
        // cout << "++++++ Ego Index +++++++  " << ext_ego_idx << endl;
        ros::spinOnce();
    }

    return 0;
}