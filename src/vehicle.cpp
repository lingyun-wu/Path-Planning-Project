#include<iostream>
#include<cmath>

#include "vehicle.hpp"


Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double v, double a) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
	
	this->lanes_available = 3;
	this->changing_lane = false;
}


vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if(state.compare("KL") == 0) {
        if (lane > 0 ) states.push_back("PLCL");
        if (lane < lanes_available-1) states.push_back("PLCR");
    } else if (state.compare("PLCL") == 0) {
            states.push_back("PLCL");
            states.push_back("LCL");
    } else if (state.compare("PLCR") == 0) {
            states.push_back("PLCR");
            states.push_back("LCR");
    }
	
    return states;
}



vector<vector<double> > Vehicle::prediction(vector<vector<double> > &sensor_fusion) {
    int s = sensor_fusion.size();

    vector<vector<double> > results;
    for (int i = 0; i < s; ++i) {
        int car_s = sensor_fusion[i][5];

}
