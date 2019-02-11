#ifndef PARAMS_HPP
#define PARAMS_HPP

#include<iostream>
#include<cmath>


const double PARAM_DETECT_RANGE = 50;  // Detection range is 60 meters around the ego vehicle 
const double PARAM_LANE_WIDTH = 4.0;   // Lane width

const int PARAM_NB_POINTS = 50;        // Output next way points number
const double PARAM_DT = 0.02;          // Time resolution


const double PARAM_MAX_SPEED = 21;      // m.s-1 
const double PARAM_MAX_ACCEL = 5;       // m.s-2
 
const double PARAM_DIST_SAFTY = 10;     // Safety distance
const double PARAM_DIST_MERGE = 15;     // Distance for merge 
const double PARAM_DIST_SAFTY_BACK = 5; // Safety back distance
const double PARAM_DIST_MERGE_BACK = 8; // Back distance for merge
const double PARAM_DIST_SPECIAL = 2;    


// Function for limit s in the range of (0, 6945.554
inline double wrap_s(double s) {
    double max_s = 6945.554;

    while (s >= max_s) {
        s -= max_s;
    }
    while (s < 0.0) {
        s += max_s;
    }
    return s;
}

#endif
