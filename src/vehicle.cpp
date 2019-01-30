#include<iostream>
#include<cmath>

#include "vehicle.hpp"
#include "params.hpp"


Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double v, double a) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
	
	this->lanes_available = 3;
	this->changing_lane = false;
    this->T = 2;    // 2 second
	this->target_speed = PARAM_MAX_SPEED;
}



Vehicle::generator_trajectory() {


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



vector<vector<double> > Vehicle::surroundings(vector<vector<double> > &sensor_fusion) {
    int size = sensor_fusion.size();

    vector<vector<double> > results;
    for (int i = 0; i < size; ++i) {
        double car_d = sensor_fusion[i][6];
        if (car_d < 0) continue;

        double car_s = sensor_fusion[i][5];
        double delta_s = car_s - this->s;
        if (abs(delta_s) < PARAM_DETECT_RANGE) {
            double car_vx = sensor_fusion[i][3];
            double car_vy = sensor_fusion[i][4];
            double car_speed = sqrt(car_vx*car_vx + car_vy*car_vy);
            results.push_back({car_d, car_speed, delta_s});
        }
    }

    return results;
}



vector<vector<vector<double> > > Vehicle::surroundings_in_order(vector<vector<double> > &predictions) {
    int size = predictions.size();
    vector<vector<vector<double> > > results(2);

    map<double, int> left, right;

    double front_position = 70.0, back_position = -70.0;
    for (int i = 0; i < size; ++i) {
        double car_d = predictions[i][0];
        int car_lane = lane_determine(car_d);
        if (car_lane < 0) continue;

        double delta_s = predictions[i][2];
        double car_speed = predictions[i][1];
        if (car_lane == lane) {
        // Get current lane's front and back car info.
            if (delta_s > 0 && delta_s < front_position) {
                front_position = delta_s;
                front_car[0] = car_speed;
                front_car[1] = delta_s;
                front_car_exist = true;
            } else if (delta_s < 0 && delta_s > back_position) {
                back_position = delta_s;
                back_car[0] = car_speed;
                back_car[1] = delta_s;
                back_car_exist = true;
            }
        } else if (car_lane == lane-1) {
        // Left lane's cars
            left[delta_s] = i;
        } else if (car_lane == lane+1) {
        // Right lane's cars
            right[delta_s] = i;
        }
    }

    for (map<double,int>::iterator it = left.begin(); it != left.end(); ++it) {
        int index = it->second;
        results[0].push_back({predictions[index][2], predictions[index][1]});
    }
    for (map<double,int>::iterator it = right.begin(); it != right.end(); ++it) {
        int index = it->second;
        results[1].push_back({predictions[index][2], predictions[index][1]});
    }

    
    if (front_position == 70.0) front_car_exist = false;
    if (back_position == -70.0) back_car_exist = false;

    return results;
}




vector<double> Vehicle::keep_lane_trajectory() {
	double accel = 0.7 * PARAM_MAX_ACCEL;
	
	double car_s = 0, car_v = 0, car_a = 0;
	if (!front_car_exist) {
		if (v + T*accel < target_speed) {
			car_s = s + v * T + 0.5 * accel * T * T;
			car_v = v + T * accel;
			car_a = 0.2 * accel;
		} else {
			car_a = 0;
			car_v = target_speed;
			double temp_t = (car_v - v) / accel;
			car_s = s + v*T + 0.5 * accel * temp_t * temp_t;
		}
	} else {
		double front_speed = front_car[0];
		double front_dist = front_car[1];
		if (front_dist < PARAM_DIST_SAFTY) {
			car_v = v - accel * T;
			if (car_v < 0) car_v = 0;
			car_a = 0;
			car_s = s + front_speed * T + front_dist - PARAM_DIST_SAFTY;
			if (car_s < s) car_s = s + front_speed * T + front_dist - PARAM_DIST_SPECIAL;
		} else {
			double dist = front_dist - PARAM_DIST_SAFTY;
			if (dist > v*T+0.5*accel*T*T-front_speed*T) {
				car_s = s + v * T + 0.5 * accel * T * T * (front_speed>v?1:(-1));
				car_v = v + accel*T*(front_speed>v?1:(-1));
				car_a = 0;
			} else {
				car_s = s + front_dist + front_speed * T - PARAM_DIST_SAFTY;
				car_v = car_speed;
				car_a = 0;
			}
		}
	}

    return {car_s, car_v, car_a};
}







int Vehicle::lane_determine(double car_d) {
    int result = -1;
    if (car_d >= 0 && car_d < 4) result = 0;
    else if (car_d >= 4 && car_d < 8) result = 1;
    else if (car_d >= 8 && car_d < 12) result = 2;
    
    return result;
}
