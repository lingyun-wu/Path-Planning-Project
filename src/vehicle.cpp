
#include "vehicle.hpp"


vehicle::vehicle() {}

vehicle::vehicle(int lane, double s, double d, double v) {

    this->lane = lane;
    this->s = s;
    this->d = d;
    this->v = v;
    this->state = state;
	
	this->lanes_available = 3;
	this->changing_lane = false;
    this->T = 2;    // 2 second
	this->target_speed = PARAM_MAX_SPEED;

    (this->front_car).resize(2);
    (this->back_car).resize(2);
    this->front_car_exist = false;
    this->back_car_exist = false;
}



vehicle::~vehicle() {}


vector<vector<double> > vehicle::generate_trajectory(vector<vector<double> > const &sensor_fusion, int n) {
	
    int restart_n = PARAM_NB_POINTS - n - 1;
    
    vector<vector<double> > results(2); 

    vector<vector<double> > surround_cars = surroundings(sensor_fusion);
    vector<vector<vector<double> > >  two_lane_predictions = surroundings_in_order(surround_cars);

    vector<double> end_s = keep_lane_trajectory();
	vector<double> start_s(3, 0);
    
    vector<double> end_d = {d, 0, 0};
    vector<double> start_d = {d, 0, 0};

	if (previous_s.size() == 0) {
		start_s[0] = s;
	} else {
        start_s[0] = previous_s[restart_n].f;
        start_s[1] = previous_s[restart_n].f_dot;
        start_s[2] = previous_s[restart_n].f_ddot;
    }
  
    cout << "start " << start_s[0] << ' ' << start_s[1] << ' ' << start_s[2] << endl;
    results[0] = JMT(start_s, end_s, T);
    results[1] = JMT(start_d, end_d, T);

    vector<Point3> new_s;
    vector<Point3> new_d;

    double dt = PARAM_DT;
    for (int i = 0; i < PARAM_NB_POINTS; ++i) {
        double car_s = polyeval(results[0], dt);
        double car_s_dot = polyeval_dot(results[0], dt);
        double car_s_ddot = polyeval_ddot(results[0], dt);

        double car_d = polyeval(results[1], dt);
        double car_d_dot = polyeval_dot(results[1], dt);
        double car_d_ddot = polyeval_dot(results[1], dt);

        new_s.push_back(Point3(car_s, car_s_dot, car_s_ddot));
        new_d.push_back(Point3(car_d, car_d_dot, car_d_ddot));

		dt += PARAM_DT;
	}
    
    previous_s = new_s;
    previous_d = new_d;

	return results;
}






vector<string> vehicle::successor_states() {
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



vector<vector<double> > vehicle::surroundings(vector<vector<double> > const &sensor_fusion) {
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



vector<vector<vector<double> > > vehicle::surroundings_in_order(vector<vector<double> > &predictions) {
    int size = predictions.size();
    vector<vector<vector<double> > > results(2);

    map<double, int> left, right;

    this->front_car_exist = false;
    this->back_car_exist = false;
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
   
    return results;
}




vector<double> vehicle::keep_lane_trajectory() {
	double accel = 0.9 * PARAM_MAX_ACCEL;
	
	double car_s = 0, car_v = 0, car_a = 0;
	if (!front_car_exist) {
		car_a = 0;
		car_v = target_speed;
		car_s = s + T * 0.5 * (v + car_v);	
	} else {
		double front_speed = front_car[0];
		double front_dist = front_car[1];
	    
        car_v = front_speed > target_speed ? target_speed : front_speed;
        car_a = 0;
        if (front_dist - T*0.5*(v+car_v) + front_speed*T < PARAM_DIST_SAFTY) {
            car_s = s + front_dist + front_speed*T - PARAM_DIST_SAFTY;
        } else {
            car_s = s + T*0.5*(v+car_v);
        }
    }

    return {car_s, car_v, car_a};
}







int vehicle::lane_determine(double car_d) {
    int result = -1;
    if (car_d >= 0 && car_d < 4) result = 0;
    else if (car_d >= 4 && car_d < 8) result = 1;
    else if (car_d >= 8 && car_d < 12) result = 2;
    
    return result;
}





vector<double> vehicle::JMT(vector<double> &start, vector<double> &end, double T) {
	/**
	 * Calculate the Jerk Minimizing Trajectory that connects the initial state
	 * to the final state in time T.
	 *
	 * @param start - the vehicles start location given as a length three array
	 *   corresponding to initial values of [s, s_dot, s_double_dot]
	 * @param end - the desired end state for vehicle. Like "start" this is a
	 *   length three array.
	 * @param T - The duration, in seconds, over which this maneuver should occur.
	 *
	 * @output an array of length 6, each value corresponding to a coefficent in 
	 *   the polynomial:
	 *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
	 *
	 * EXAMPLE
	 *   > JMT([0, 10, 0], [10, 10, 0], 1)
	 *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
	 */
	MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
	  3*T*T, 4*T*T*T,5*T*T*T*T,
	  6*T, 12*T*T, 20*T*T*T;

	MatrixXd B = MatrixXd(3,1);     
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
	  end[1]-(start[1]+start[2]*T),
	  end[2]-start[2];

	MatrixXd Ai = A.inverse();

	MatrixXd C = Ai*B;

	vector <double> result = {start[0], start[1], .5*start[2]};

	for(int i = 0; i < C.size(); ++i) {
		result.push_back(C.data()[i]);
	}

	return result;
}



double vehicle::polyeval(vector<double> c, double t) {
  double res = 0.0;
  for (size_t i = 0; i < c.size(); i++) {
    res += c[i] * pow(t, i);
  }
  return res;
}

// 1st derivative of a polynom
double vehicle::polyeval_dot(vector<double> c, double t) {
  double res = 0.0;
  for (size_t i = 1; i < c.size(); ++i) {
    res += i * c[i] * pow(t, i-1);
  }
  return res;
}

// 2nd derivative of a polynom
double vehicle::polyeval_ddot(vector<double> c, double t) {
  double res = 0.0;
  for (size_t i = 2; i < c.size(); ++i) {
    res += i * (i-1) * c[i] * pow(t, i-2);
  }
  return res;
}
