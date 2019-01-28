#ifndef VEICLE_H
#define VEHICLE_H

#include<iostream>
#include<random>
#include<vector>
#include<map>
#include<string>

using namespace std;

class Vehicle {

	public:
		
		int lane;
		double s;
		double v;
		double a;

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

}




#endif
