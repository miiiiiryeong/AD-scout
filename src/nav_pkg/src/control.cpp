#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>

#include "control.h"
#include "extern_variables.h"

using namespace std;

// �ϴ� local_path�� global path ���� ���� �ε����κ��� ���� �ε�����ŭ �߶� ���� path
void Control::PurePursuit(vector < vector <double> > local_path) {
    // cout << "pure pursuit is working!!" << "\n";
    lookahead = 6; //
    target_index = lookahead * 10;

    double target_x = 0;
    double target_y = 0;
    // cout << "local path size: " << local_path.size() << endl;

    if (target_index < local_path.size()) {
        // while (true) {
        //     // cout << "local path size: " << local_path.size() << endl;
        //     target_x = local_path[target_index][0];
        //     target_y = local_path[target_index][1];
        //     // cout << "if 1" << endl;
        //     if (lookahead - lookahead * 1 / 3 < sqrt(pow(target_x - ext_ego_x, 2) + pow(target_y - ext_ego_y, 2)) && sqrt(pow(target_x - ext_ego_x, 2) + pow(target_y - ext_ego_y, 2)) < lookahead + lookahead * 1 / 3) break;
        //     else if (sqrt(pow(target_x - ext_ego_x, 2) + pow(target_y - ext_ego_y, 2)) < lookahead) {
        //         target_index++;
        //     }
        //     else if (sqrt(pow(target_x - ext_ego_x, 2) + pow(target_y - ext_ego_y, 2)) > lookahead) {
        //         if (target_index > 0) target_index--;
        //     }
        //     else break;
        // }
        target_x = local_path[target_index][0];
        target_y = local_path[target_index][1];
        // cout << "target_index : " << target_index << endl;
        // cout << "target_x : " << target_x << endl;
        // cout << "target_y : " << target_y << endl;
    }
    else {
        int last_x = local_path.size() - 1;
        // cout << "1" << endl;
        target_x = local_path[last_x][0];
        // cout << "2" << endl;

        int last_y = local_path.size() - 1;
        target_y = local_path[last_y][1];
    }

    double tmp = fmod(atan2(target_y - ext_ego_y, target_x - ext_ego_x) * (180.0 / pi), 360.0);

    // cout << "heading : " << ext_ego_heading << endl;
    double alpha = ext_ego_heading - tmp;


    angle = atan2(2.0 * WB * sin(alpha * (pi / 180.0)) / lookahead, 1.0);
    // cout << "degree angle is : " << angle * (180.0 / pi) << endl;
}

void Control::LateralController() {

    double sampling_time = 0.1; // need to be determined(considering hz of ros msg)

    double target_ang_vel = angle / sampling_time;

    ext_angular_velocity_input = ext_ego_cur_angular_velocity + (target_ang_vel - ext_ego_cur_angular_velocity); // angular velocity input
    // cout << "angular velocity : " << ext_angular_velocity_input << "\n";
}

void Control::LongitudinalController(double target_velocity) {
    ext_linear_velocity_input = target_velocity / 3.6;
}

void Control::run(vector < vector <double> > local_path, double target_velocity){
    PurePursuit(local_path);
    LateralController();
    LongitudinalController(target_velocity);
}