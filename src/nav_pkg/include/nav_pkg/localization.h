#pragma once
#include <cmath>
#include <vector>
#include <fstream>
#include <string>
using namespace std;

class Local
{
public:
    struct Waypoint {
        double x;
        double y;
        double index;
        string drivingInfo;
    };

    double x = 0;
    double y = 0;
    double z = 0;

    std::vector<std::vector<double>> global_map;
    string STRING_driving_info[69574][1];

    void readCSV();
    void findClosestWaypoint();
};