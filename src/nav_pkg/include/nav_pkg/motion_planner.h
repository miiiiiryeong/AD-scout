#pragma once
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>

using namespace std;

class MotionPlanner {
public:
    vector < vector <double> > self_path;

    void find_local_path(std::vector<std::vector<double>> global_map);
};