#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include<iostream>
#include<random>
#include<vector>
#include<map>
#include<string>
#include "Eigen-3.3/Eigen/Dense"

#include"params.hpp"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;



// Structure contains position, speed and acceleration
struct Point3 {
    double f;
    double f_dot;
    double f_ddot;
    Point3 (double y=0, double y_dot=0, double y_ddot=0): f(y), f_dot(y_dot), f_ddot(y_ddot) {};
};



// Vehicle class
class vehicle {

	public:
		
		int lane;              // car's lane
        int target_lane;       // car's target lane
        int lanes_available;   // number of lanes
        double s;              
        double d;
		double v;

        double T;               // prediction time 

		double target_speed;    // target speed

		string state;           // current state

        bool front_car_exist;
        bool back_car_exist;
        vector<double> front_car;    // front car info.
        vector<double> back_car;     // back car info.

        vector<vector<double> > coeffs;     // prediction trajectory 


		// Constructor		
		vehicle();
		vehicle(int lane, double s, double v);
	
        // Destructor
        virtual ~vehicle();

        // Get trajectory
		vector<vector<double> > generate_trajectory(vector<vector<double> > const &sensor_fusion, int n);
        // Get next possible states
        vector<string> successor_states();
        // Get nearby cars
        vector<vector<double> > surroundings(vector<vector<double> > const &sensor_fusion);
        // Get neighbor cars in order
        vector<vector<vector<double> > > surroundings_in_order(vector<vector<double> > &predictions);

        // Keep lane
        Point3 keep_lane_trajectory(Point3 start_s);
        // Change lane
        vector<Point3> lane_change_trajectory(string st, Point3 start_s, Point3 start_d);
        // Prepare lane change        
        double prepare_lane_change_trajectory(string st, Point3 start_s, Point3 &end_s, vector<vector<vector<double> > > &predictions);
        // Check collsion if it may happen
        bool check_collision(string st, vector<vector<vector<double> > > &predictions);

        // Get lane number
        int lane_determine(double car_d);
        // Do JMT
        vector<double> JMT(Point3 &Start, Point3 &End, double T);

        // Get position
        double polyeval(vector<double> c, double t);
        // Get speed
        double polyeval_dot(vector<double> c, double t);
        // Get acceleration
        double polyeval_ddot(vector<double> c, double t);
};




#endif
