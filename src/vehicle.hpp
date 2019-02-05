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




struct Point3 {
    double f;
    double f_dot;
    double f_ddot;
    Point3 (double y=0, double y_dot=0, double y_ddot=0): f(y), f_dot(y_dot), f_ddot(y_ddot) {};
};




class vehicle {

	public:
		
		int lane;
        int lanes_available;
        double s;
        double d;
		double v;

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

        int start_n;
        vector<vector<double> > coeffs;


		/**
		 * Constructor
		**/

		vehicle();
		vehicle(int lane, double s, double v);
	
        virtual ~vehicle();

		vector<vector<double> > generate_trajectory(vector<vector<double> > const &sensor_fusion, int n);
        vector<string> successor_states();
        vector<vector<double> > surroundings(vector<vector<double> > const &sensor_fusion);
        vector<vector<vector<double> > > surroundings_in_order(vector<vector<double> > &predictions);
        Point3 keep_lane_trajectory(Point3 start_s);


        int lane_determine(double car_d);
        vector<double> JMT(Point3 &Start, Point3 &End, double T);

        double polyeval(vector<double> c, double t);
        double polyeval_dot(vector<double> c, double t);
        double polyeval_ddot(vector<double> c, double t);
};




#endif
