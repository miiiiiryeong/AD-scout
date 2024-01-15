#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>

#include "control.h"
#include "extern_variables.h"

using namespace std;

// �ϴ� local_path�� global path ���� ���� �ε����κ��� ���� �ε�����ŭ �߶� ���� path
void Control::PurePursuit(vector < vector <double> > local_path) {
    lookahead = 7.5; // speed == 38kph�� ��
    target_index = lookahead * 10;

    double target_x = 0;
    double target_y = 0;

    if (target_index < local_path.size()) {
        while (true) {
            target_x = local_path[target_index][0];
            target_y = local_path[target_index][1];
            if (lookahead - lookahead * 1 / 3 < sqrt(pow(target_x - ext_ego_x, 2) + pow(target_y - ext_ego_y, 2)) && sqrt(pow(target_x - ext_ego_x, 2) + pow(target_y - ext_ego_y, 2)) < lookahead + lookahead * 1 / 3) break;
            else if (sqrt(pow(target_x - ext_ego_x, 2) + pow(target_y - ext_ego_y, 2)) < lookahead) {
                target_index++;
            }
            else if (sqrt(pow(target_x - ext_ego_x, 2) + pow(target_y - ext_ego_y, 2)) > lookahead) {
                target_index--;
            }
            else break;
        }
    }
    else {
        int last_x = local_path.size() - 1;
        target_x = local_path[last_x][0];

        int last_y = local_path.size() - 1;
        target_y = local_path[last_y][1];
    }

    double tmp = fmod(atan2(target_y - ext_ego_y, target_x - ext_ego_x) * (180.0 / pi), 360.0);

    // cout << "heading : " << ext_ego_heading << endl;
    double alpha = ext_ego_heading - tmp;

    angle = atan2(2.0 * 2.7 * sin(alpha * (pi / 180.0)) / lookahead, 1.0);
}

void Control::LateralController() {

    double sampling_time = 0.1; // need to be determined(considering hz of ros msg)

    double target_ang_vel = angle / sampling_time;

    ext_angular_velocity_input = ext_ego_cur_angular_velocity + (target_ang_vel - ext_ego_cur_angular_velocity); // angular velocity input
}

void Control::LongitudinalController(double target_velocity) {
    ext_linear_velocity_input = target_velocity / 3.6;
}

void Control::run(vector < vector <double> > local_path, double target_velocity){
    PurePursuit(local_path);
    LateralController();
    LongitudinalController(target_velocity);
}