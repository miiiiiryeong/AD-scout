#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <string>
using namespace std;

class Control {
public:
    double pi = 3.14159265358979323846;
    double lookahead = 0;
    double alpha = 0;
    int target_index = 0;
    double angle = 0;

    void PurePursuit(vector < vector <double> > local_path);

    void LongitudinalController(double target_velocity);
    void LateralController();
    void run(vector < vector <double> > local_path, double target_velocity);
};