#include <iostream>
#include "ros/ros.h"
#include <cmath>
#include <string>
#include <vector>

#include "extern_variables.h"
#include "scout_msgs/scout_status.h" // �´�...?
#include "scout_msgs/cmd_vel.h" // �´�...?
#include <nav_msgs/Odometry>

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

void LocalizationCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    ext_ego_x = msg->pose.pose.position.x;
    ext_ego_y = msg->pose.pose.position.y;
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw = yaw * (180 / M_PI);
    ext_ego_heading = fmod(yaw, 360.0);
    if (ext_ego_heading < 0){
        ext_ego_heading += 360;
    }
}

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
    ros::Subscriber loc_sub = nh.subscribe("/odometry", 10, LocalizationCallback);

    local.readCSV();

    while (ros::ok())
    {
        ctrl_cmd_msg.linear.x = ext_linear_velocity_input;
        ctrl_cmd_msg.angular.z = ext_angular_velocity_input;
        ctrl_cmd_pub.publish(ctrl_cmd_msg);
        egoTopicFlag = false;

        local.findClosestWaypoint();
        motion.find_local_path(local.global_map);
        control.run(motion.self_path, 5);

        // cout << "target_velocity : " << ext_linear_velocity_input << endl;
        // cout << "ego velocity : " << ext_ego_velocity << endl;
        // cout << "State : " << ext_ego_state << "  Behavior : " << ext_ego_behavior << endl;
        // cout << "++++++ Ego Index +++++++  " << ext_ego_idx << endl;
        ros::spinOnce();
    }

    return 0;
}