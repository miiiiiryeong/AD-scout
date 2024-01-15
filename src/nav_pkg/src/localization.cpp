#include <iostream>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <fstream>
#include <string>

#include "extern_variables.h"
#include "localization.h"
using namespace std;

void Local::readCSV() {
    vector<Waypoint> waypoints;
    const string filename = "/home/mino/AD-scout/global_path.csv";
    ifstream file(filename);

    if (!file.is_open()) {
        cerr << "Error: Could not open CSV file." << endl;
    }

    string line;
    int i = 0;
    while (getline(file, line)) {
        istringstream iss(line);
        Waypoint waypoint;
        char delimiter = ',';
        if (iss >> waypoint.x >> delimiter >> waypoint.y >> delimiter >> waypoint.index) {
            waypoints.push_back(waypoint);

            // ���Ϳ� ��(���� ����) �߰�
            global_map.push_back(std::vector<double>(3));
            global_map[i][0] = double(waypoint.x);
            global_map[i][1] = double(waypoint.y);
            global_map[i][2] = double(waypoint.index);
        }
        else {
            cerr << "Error: Invalid line in CSV file: " << line << endl;
        }
        i++;
    }
    file.close();
}

void Local::findClosestWaypoint() {
    int closestIndex = 0;
    double minDistance = -1;
    int save_index = 0;

    for (int i = 0; i < global_map.size(); i++) {
        double distance = sqrt(pow(ext_ego_x - global_map[i][0], 2) + pow(ext_ego_y - global_map[i][1], 2));
        if ((minDistance > distance || minDistance == -1) && save_index <= i) {
            minDistance = distance;
            // cout << "global path index : " << i << endl;
            closestIndex = i;
            save_index = i;
        }
    }
    ext_ego_idx = closestIndex;
    // cout << "my index : " << closestIndex << endl;
}