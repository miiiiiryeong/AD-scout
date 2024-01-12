#pragma once
#include <iostream>
#include <string>
using namespace std;

// input
extern double ext_angular_velocity_input;
extern double ext_linear_velocity_input;

// from scout
extern double ext_ego_cur_linear_velocity;
extern double ext_ego_cur_angular_velocity;

extern double ext_ego_heading;
extern double ext_ego_x;
extern double ext_ego_y;
extern int ext_ego_idx;
extern double ext_ego_target_steering;
extern string ext_ego_state;
extern string ext_ego_behavior;
extern int ext_ego_lane_cnt;
extern int ext_ego_selected_lane;