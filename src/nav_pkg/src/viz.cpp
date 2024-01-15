#include <iostream>
#include <tf2_ros/transform_broadcaster.h>
#include "math.h"
// visualization용 code
#include "nav_msgs/Path.h"
#include <ctime>
#include "ros/ros.h"
#include <cmath>
#include <string>
#include <vector>
#include <tf/tf.h>

#include "extern_variables.h"
#include "scout_msgs/ScoutStatus.h" // �´�...?
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include "control.h"
#include "localization.h"
#include "motion_planner.h"
using namespace std;

Local viz_local;
Control viz_control;
MotionPlanner viz_motion;


// Visualization
nav_msgs::Odometry viz_position;
sensor_msgs::PointCloud viz_trajectory;
geometry_msgs::Point32 ppoint;
double t = time(NULL);
nav_msgs::Path viz_global_path;
nav_msgs::Path gp;
nav_msgs::Path viz_self_path;
nav_msgs::Path viz_lattice_path_0;
nav_msgs::Path viz_lattice_path_1;
nav_msgs::Path viz_lattice_path_2;
nav_msgs::Path viz_lattice_path_3;
nav_msgs::Path viz_lattice_path_4;
nav_msgs::Path lp_self;
nav_msgs::Path lp0;
nav_msgs::Path lp1;
nav_msgs::Path lp2;
nav_msgs::Path lp3;
nav_msgs::Path lp4;
// visualization_msgs::MarkerArray viz_lidar;
// visualization_msgs::Marker circle_marker;
/////////////////////////////

void Visualizer(){
    // position visualization

    ppoint.x = ext_ego_x;
    // cout << "ego x : " << ext_ego_x << endl;
    ppoint.y = ext_ego_y;
    ppoint.z = 0;

    viz_position.header.frame_id = "camera_init";
    viz_trajectory.header.frame_id = "camera_init";
    viz_position.pose.pose.position.x = ppoint.x;
    viz_position.pose.pose.position.y = ppoint.y;
    viz_trajectory.header.stamp = ros::Time::now();
    if (t - time(NULL) < 0.1)
    {
        t = time(NULL);
        viz_trajectory.points.push_back(ppoint);
    } 


}

// void egoTopicCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg){
//     ext_ego_velocity = msg->velocity.x * 3.6;
// }

void global_path_Visualizer(){
    // cout << "global_path_Visualizer" << endl;
    // global path visualization
    viz_global_path.header.frame_id = "camera_init";
    for (int i = 0; i <viz_local.global_map.size(); i++){
        geometry_msgs::PoseStamped read_pose;
        read_pose.pose.position.x = viz_local.global_map[i][0];
        read_pose.pose.position.y = viz_local.global_map[i][1];
        read_pose.pose.position.z = 0;
        read_pose.pose.orientation.x = 0;
        read_pose.pose.orientation.y = 0;
        read_pose.pose.orientation.z = 0;
        read_pose.pose.orientation.w = 1;
        gp.poses.push_back(read_pose);
    }
    viz_global_path.poses = gp.poses;


}

void lattice_path_Visualizer(){
    viz_self_path.header.frame_id = "camera_init";
    // cout << "starting lattice_path_Visualizer" << endl;
    viz_lattice_path_0.header.frame_id = "camera_init";
    viz_lattice_path_1.header.frame_id = "camera_init";
    viz_lattice_path_2.header.frame_id = "camera_init";
    viz_lattice_path_3.header.frame_id = "camera_init";
    viz_lattice_path_4.header.frame_id = "camera_init";

    lp_self.poses.clear();
    viz_self_path.poses.clear();

    for (int i = 0; i < viz_motion.self_path.size(); i++){ // 지금 0 들어감
        geometry_msgs::PoseStamped read_pose_self;
        read_pose_self.pose.position.x = viz_motion.self_path[i][0];
        // cout << "self path : "<< viz_motion.self_path[i][0] << endl;
        read_pose_self.pose.position.y = viz_motion.self_path[i][1];
        read_pose_self.pose.position.z = 0;
        read_pose_self.pose.orientation.x = 0;
        read_pose_self.pose.orientation.y = 0;
        read_pose_self.pose.orientation.z = 0;
        read_pose_self.pose.orientation.w = 1;
        lp_self.poses.push_back(read_pose_self);
    }
    viz_self_path.poses = lp_self.poses; // cut path from global path
    // cout << "after self_path_Visualize" << endl;

    // lp0.poses.clear();
    // // viz_lattice_path_0.poses.clear();
    // viz_lattice_path_0.poses.clear();
    // for (int i = 0; i < viz_motion.lattice[0].first.size(); i++){
    //     geometry_msgs::PoseStamped read_pose0;
    //     if(viz_motion.lattice[0].first[i] > 0 || viz_motion.lattice[0].second[i] > 0){
    //     read_pose0.pose.position.x = viz_motion.lattice[0].first[i];
    //     read_pose0.pose.position.y = viz_motion.lattice[0].second[i];
    //     read_pose0.pose.position.z = 0;
    //     read_pose0.pose.orientation.x = 0;
    //     read_pose0.pose.orientation.y = 0;
    //     read_pose0.pose.orientation.z = 0;
    //     read_pose0.pose.orientation.w = 1;
    //     lp0.poses.push_back(read_pose0);
    //     }
    // }
    // viz_lattice_path_0.poses = lp0.poses;
    // // cout << "after lattice path 0_Visualize" << endl;

    // lp1.poses.clear();
    // viz_lattice_path_1.poses.clear();

    // // cout << "lattice size : "<<viz_motion.lattice.size() << endl;
    // for (int i = 0; i < viz_motion.lattice[1].first.size(); i++){
    //     geometry_msgs::PoseStamped read_pose1;
    //     if(viz_motion.lattice[1].first[i] > 0 || viz_motion.lattice[1].second[i] > 0){
    //     read_pose1.pose.position.x = viz_motion.lattice[1].first[i];
    //     read_pose1.pose.position.y = viz_motion.lattice[1].second[i];
    //     read_pose1.pose.position.z = 0;
    //     read_pose1.pose.orientation.x = 0;
    //     read_pose1.pose.orientation.y = 0;
    //     read_pose1.pose.orientation.z = 0;
    //     read_pose1.pose.orientation.w = 1;
    //     lp1.poses.push_back(read_pose1);
    //     }
    // }
    // viz_lattice_path_1.poses = lp1.poses;

    // lp2.poses.clear();
    // viz_lattice_path_2.poses.clear();
    // for (int i = 0; i < viz_motion.lattice[2].first.size(); i++){
    //     geometry_msgs::PoseStamped read_pose2;
    //     if(viz_motion.lattice[2].first[i] > 0 || viz_motion.lattice[2].second[i] > 0){
    //     read_pose2.pose.position.x = viz_motion.lattice[2].first[i];
    //     read_pose2.pose.position.y = viz_motion.lattice[2].second[i];
    //     read_pose2.pose.position.z = 0;
    //     read_pose2.pose.orientation.x = 0;
    //     read_pose2.pose.orientation.y = 0;
    //     read_pose2.pose.orientation.z = 0;
    //     read_pose2.pose.orientation.w = 1;
    //     lp2.poses.push_back(read_pose2);
    //     }
    // }
    // viz_lattice_path_2.poses = lp2.poses;

    // cout << "after lattice path 1_Visualize" << endl;

    // lp3.poses.clear();
    // viz_lattice_path_3.poses.clear();

    // for (int i = 0; i < viz_motion.lattice[3].first.size(); i++){
    //     geometry_msgs::PoseStamped read_pose3;
    //     if(viz_motion.lattice[3].first[i] > 0 || viz_motion.lattice[3].second[i] > 0){
    //     read_pose3.pose.position.x = viz_motion.lattice[3].first[i];
    //     read_pose3.pose.position.y = viz_motion.lattice[3].second[i];
    //     read_pose3.pose.position.z = 0;
    //     read_pose3.pose.orientation.x = 0;
    //     read_pose3.pose.orientation.y = 0;
    //     read_pose3.pose.orientation.z = 0;
    //     read_pose3.pose.orientation.w = 1;
    //     lp3.poses.push_back(read_pose3);
    //     }
    // }
    // viz_lattice_path_3.poses = lp3.poses;

    // lp4.poses.clear();
    // viz_lattice_path_4.poses.clear();
    
    // for (int i = 0; i < viz_motion.lattice[4].first.size(); i++){
    //     geometry_msgs::PoseStamped read_pose4;
    //     if(viz_motion.lattice[4].first[i] > 0 || viz_motion.lattice[4].second[i] > 0){
    //     read_pose4.pose.position.x = viz_motion.lattice[4].first[i];
    //     read_pose4.pose.position.y = viz_motion.lattice[4].second[i];
    //     read_pose4.pose.position.z = 0;
    //     read_pose4.pose.orientation.x = 0;
    //     read_pose4.pose.orientation.y = 0;
    //     read_pose4.pose.orientation.z = 0;
    //     read_pose4.pose.orientation.w = 1;
    //     lp4.poses.push_back(read_pose4);
    //     }
    // }
    // viz_lattice_path_4.poses = lp4.poses;

}

// void lidar_visualizer(){
//     int c_id1 = 0;

//     for (int i = 0; i < viz_shared.enu_obstacle_x.size(); i++){
//         visualization_msgs::Marker circle_marker;
//         circle_marker.header.frame_id = "camera_init";
//         circle_marker.ns = "circles";
//         circle_marker.id = c_id1;
//         circle_marker.type = visualization_msgs::Marker::CYLINDER;
//         circle_marker.action = visualization_msgs::Marker::ADD;
//         circle_marker.pose.position.z = -0.1;
//         circle_marker.pose.orientation.x = 0.0;
//         circle_marker.pose.orientation.y = 0.0;
//         circle_marker.pose.orientation.z = 0.0;
//         circle_marker.pose.orientation.w = 1.0;
//         circle_marker.scale.z = 0.1;
//         circle_marker.color.r = 0.2;
//         circle_marker.color.g = 0.8;
//         circle_marker.color.b = 0.2;
//         circle_marker.color.a = 1.0;
//         circle_marker.lifetime = ros::Duration(0.1);
//         circle_marker.pose.position.x = viz_shared.enu_obstacle_x[i];
//         circle_marker.pose.position.y = viz_shared.enu_obstacle_y[i];
//         circle_marker.scale.x = viz_shared.obstacle_depth[i];
//         circle_marker.scale.y = viz_shared.obstacle_width[i];
//         viz_lidar.markers.push_back(circle_marker);
//         c_id1 = c_id1 + 1;
//     }
// }

void rviz_tf_publisher(){
    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();

    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "camera_init";
    // Here we set camera_init to be 1 meter to the right of map frame in x direction.
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    tf_broadcaster.sendTransform(transformStamped);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "viz_node");
    ros::NodeHandle nh;

    ros::Publisher position_viz_pub = nh.advertise<nav_msgs::Odometry>("/viz_position", 1);
    ros::Publisher trajectory_viz_pub = nh.advertise<sensor_msgs::PointCloud>("/viz_trajectory", 1);
    ros::Publisher global_path_viz_pub = nh.advertise<nav_msgs::Path>("/viz_global_path", 1);
    // ros::Publisher lattice_path_0_viz_pub = nh.advertise<nav_msgs::Path>("/viz_lattice_path_0", 1);
    // ros::Publisher lattice_path_1_viz_pub = nh.advertise<nav_msgs::Path>("/viz_lattice_path_1", 1);
    // ros::Publisher lattice_path_2_viz_pub = nh.advertise<nav_msgs::Path>("/viz_lattice_path_2", 1);
    ros::Publisher self_path_viz_pub = nh.advertise<nav_msgs::Path>("/viz_self_path", 1);
    // ros::Publisher lidar_pub = nh.advertise<visualization_msgs::MarkerArray>("/viz_lidar", 1);
    // ros::Publisher lattice_path_3_viz_pub = nh.advertise<nav_msgs::Path>("/viz_lattice_path_3", 1);
    // ros::Publisher lattice_path_4_viz_pub = nh.advertise<nav_msgs::Path>("/viz_lattice_path_4", 1);

    // ros::Subscriber gps_sub = nh.subscribe("/gps", 100, gpsCallback);
    // ros::Subscriber imu_sub = nh.subscribe("/imu", 100, imuCallback);
    // ros::Subscriber lidar_sub = nh.subscribe("/lidar3D", 100, LiDARCallback);
    // ros::Subscriber ego_topic_sub = nh.subscribe("/Ego_topic", 10, egoTopicCallback);

    viz_local.readCSV();
    global_path_Visualizer();
 

    while (ros::ok())
    {
        // viz_local.findClosestWaypoint();
        // viz_motion.find_local_path(viz_local.DOUBLE_map);
        // viz_motion.select_trajectory();
        // viz_motion.path_maker();
        // cout << "lattice : "<< viz_motion.lattice[0].first.size() << endl;

        // viz_mission.run(viz_shared.obstacle_distance, obstacle_flag);
        
        // viz_behavior.run();
        
        // viz_motion.run(viz_local.DOUBLE_map, viz_shared.obstacle_width, viz_shared.obstacle_depth, viz_shared.enu_obstacle_x, viz_shared.enu_obstacle_y);
        // viz_control.run(viz_motion.lattice, viz_motion.self_path);


        // cout << "ego index is  : " << ext_ego_idx << endl;
        // cout << "ego x is  : " << ext_ego_x << endl;
        // cout << "ego y is  : " << ext_ego_y << endl;

        rviz_tf_publisher();
        Visualizer();
        lattice_path_Visualizer();

        viz_local.findClosestWaypoint();
        // cout << "closest waypoint : " << ext_ego_idx << endl;
        viz_motion.find_local_path(viz_local.global_map);
        viz_control.run(viz_motion.self_path, 1);
        
        // Visualization
        viz_position.header.stamp = ros::Time::now();
        position_viz_pub.publish(viz_position);

        viz_trajectory.header.stamp = ros::Time::now();
        trajectory_viz_pub.publish(viz_trajectory);

        viz_global_path.header.stamp = ros::Time::now();
        global_path_viz_pub.publish(viz_global_path);

        // viz_lattice_path_0.header.stamp = ros::Time::now();
        
        // lattice_path_0_viz_pub.publish(viz_lattice_path_0);

        // viz_lattice_path_1.header.stamp = ros::Time::now();
        
        // lattice_path_1_viz_pub.publish(viz_lattice_path_1);

        // viz_lattice_path_2.header.stamp = ros::Time::now();
       
        // lattice_path_2_viz_pub.publish(viz_lattice_path_2);

        viz_self_path.header.stamp = ros::Time::now();
        
        self_path_viz_pub.publish(viz_self_path);
        
        // lidar_pub.publish(viz_lidar);

        // viz_lattice_path_3.header.stamp = ros::Time::now();
        
        // lattice_path_3_viz_pub.publish(viz_lattice_path_3);

        // viz_lattice_path_4.header.stamp = ros::Time::now();
        
        // lattice_path_4_viz_pub.publish(viz_lattice_path_4);
        ////////////////////////////////////

        ros::spinOnce();
    }
    return 0;
}