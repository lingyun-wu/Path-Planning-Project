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

        double T;

        bool changing_lane;
		double target_speed;
		double goal_s;
		double goal_lane;

		string state;

        bool front_car_exist;
        bool back_car_exist;
        vector<double> front_car;
        vector<double> back_car;

		/**
		 * Constructor
		**/

		Vehicle();
		Vehicle(int lane, double s, double v, double a);
		
		vector<string> successor_states();
        vector<vector<double> > surroundings(vector<vector<double> > &sensor_fusion);
        vector<vector<vector<double> > > surroundings_in_order(vector<vector<double> > &predictions);
        vector<double> keep_lane_trajecotry();


        int lane_determine(double car_d);
};




#endif
