#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>

#include "motion_planner.h"

void MotionPlanner::find_local_path(std::vector<std::vector<double>> global_map) {
    // NO PROBLEM
    self_path.clear();
    int end_local_index = 0;
    // cout << "map size : "<<(sizeof(global_map)/sizeof(global_map[0])) << endl;
    // cout << "global_map size 99 : " << global_map.size() << endl;
    if (ext_ego_idx + 100 > global_map.size()) {
        // cout << "map size : "<<(sizeof(global_map)/sizeof(global_map[0])) << endl;
        end_local_index = global_map.size();
    }
    else {
        end_local_index = ext_ego_idx + 100;
    }
    // cout << "ego index : "<<ext_ego_idx<<endl;
    // cout << "end local index : "<< end_local_index <<endl;
    for (int i = ext_ego_idx; i < end_local_index; ++i) {
        self_path.push_back({ global_map[i][0], global_map[i][1] });
        // cout << "self path : " << b[i][0] <<endl;
    }
}