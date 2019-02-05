#ifndef PARAMS_HPP
#define PARAMS_HPP

#include<iostream>
#include<cmath>


// Detection range is 60 meters around the ego vehicle 
const double PARAM_DETECT_RANGE = 60;
const double PARAM_LANE_WIDTH = 4.0;

const int PARAM_NB_POINTS = 50;
const int PARAM_PATH_CUTOFF = 10;
const double PARAM_DT = 0.02;


const double PARAM_MAX_SPEED = 22; // m.s-1 
const double PARAM_MAX_ACCEL = 5; // m.s-2
const double PARAM_MAX_JERK = 10;  // m.s-3
const double PARAM_MAX_SPEED_INC = PARAM_MAX_ACCEL * PARAM_DT;

const double PARAM_DIST_SAFTY = 10; 
const double PARAM_DIST_SPECIAL = 2;


#endif
