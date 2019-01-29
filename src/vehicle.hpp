#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include<iostream>
#include<random>
#include<vector>
#include<map>
#include<string>

using namespace std;

class Vehicle {

	public:
		
		int lane;
        int lanes_available;
        double s;
		double v;
		double a;

        bool changing_lane;
		double target_speed;
		double goal_s;
		double goal_lane;

		string state;

		
		/**
		 * Constructor
		**/

		Vehicle();
		Vehicle(int lane, double s, double v, double a);
		
		vector<string> successor_states();
        vector<vector<double> > prediction(vector<vector<double> > &sensor_fusion);

};




#endif
