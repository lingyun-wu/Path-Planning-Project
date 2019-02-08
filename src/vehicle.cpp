
#include "vehicle.hpp"


vehicle::vehicle() {}

vehicle::vehicle(int lane, double s, double v) {

    this->lane = lane;
    this->target_lane = lane;
    this->s = s;
    this->d = lane * PARAM_LANE_WIDTH + PARAM_LANE_WIDTH*0.5;
    this->v = v;
    this->state = "KL";
	
	this->lanes_available = 3;
    this->T = 2.5;    // 2.5 second
    this->T_CL = T;
	this->target_speed = PARAM_MAX_SPEED;

    (this->front_car).resize(2);
    (this->back_car).resize(2);
    this->front_car_exist = false;
    this->back_car_exist = false;
}



vehicle::~vehicle() {}







vector<vector<double> > vehicle::generate_trajectory(vector<vector<double> > const &sensor_fusion, int n) {
    
    int used = PARAM_NB_POINTS - n;
    double dt = PARAM_DT;
    vector<vector<double> > results(2); 

    vector<vector<double> > surround_cars = surroundings(sensor_fusion);
    vector<vector<vector<double> > >  two_lane_predictions = surroundings_in_order(surround_cars);

    // start s 
    Point3 start_s(s, v, 0);
    Point3 start_d(d, 0, 0);
    Point3 end_s, end_d(d, 0, 0);
    
    if (n != 0) {
        start_s.f = polyeval(coeffs[0], used*dt);
        start_s.f_dot = polyeval_dot(coeffs[0], used*dt);
        start_s.f_ddot = polyeval_ddot(coeffs[0], used*dt);

        start_d.f = polyeval(coeffs[1], used*dt);
        start_d.f_dot = polyeval_dot(coeffs[1], used*dt);
        start_d.f_ddot = polyeval_ddot(coeffs[1], used*dt);
    }

    if (state == "LCL" || state == "LCR") {
        start_d.f = polyeval(coeffs[1], used*dt);
        start_d.f_dot = polyeval_dot(coeffs[1], used*dt);
        start_d.f_ddot = polyeval_ddot(coeffs[1], used*dt);
        
        vector<Point3> SD = lane_change_trajectory(state, start_s, start_d);
        end_s = SD[0];
        end_d = SD[1];

        double final_d = target_lane*4.0+2.0;
        double boundary_d = state == "LCL" ? (target_lane+1)*4.0 : target_lane*4.0;
        if ((state == "LCL" && start_d.f-boundary_d < 0) || (state=="LCR" && start_d.f-boundary_d > 0)) {
            lane = target_lane;
        }

        if (abs(start_d.f-final_d) < 0.01) {
            d = lane*4.0+2.0;
            state = "KL";
        }
    } else { 
        if (state == "PLCL" || state == "PLCR") {
            string st = state == "PLCL" ? "LCL" : "LCR";
            if (!check_collision(st, two_lane_predictions)) {
                vector<Point3> temp_end = lane_change_trajectory(st, start_s, start_d);
                end_s = temp_end[0];
                end_d = temp_end[1];
                state = st;
                target_lane = state == "LCL" ? lane-1 : lane+1;
            } else {
                Point3 temp_end_s_kl = keep_lane_trajectory(start_s);
                Point3 temp_end_s_plc;
                
                double speed_plc = prepare_lane_change_trajectory(state, start_s, temp_end_s_plc, two_lane_predictions);
                if (speed_plc < temp_end_s_kl.f_dot || !front_car_exist) {
                    end_s = temp_end_s_kl;
                } else {
                    end_s = temp_end_s_plc;
                }
            }
        } else if (state == "KL") {
            if (!front_car_exist) {
                end_s = keep_lane_trajectory(start_s);
            } else {
                vector<string> states = successor_states();
                vector<double> all_speeds;
                vector<Point3> all_end_s;

                for(int i = 0; i < states.size(); ++i) {
                    if (states[i] == "KL") {
                        Point3 temp_end_s = keep_lane_trajectory(start_s); 
                        all_speeds.push_back(temp_end_s.f_dot);
                        all_end_s.push_back(temp_end_s);
                    } else if (states[i] == "PLCL" || states[i] == "PLCR") {
                        Point3 temp_end_s;
                        double temp_speed = prepare_lane_change_trajectory(states[i], start_s, temp_end_s, two_lane_predictions);
                        all_speeds.push_back(temp_speed);
                        all_end_s.push_back(temp_end_s);
                    }
                    //cout << states[i] << ' ' << all_speeds[i] << endl;
                }
                vector<double>::iterator fast = max_element(begin(all_speeds), end(all_speeds));
                int best_idx = distance(begin(all_speeds), fast);
                state = states[best_idx];
                end_s = all_end_s[best_idx];

            }
        }

    }

    results[0] = JMT(start_s, end_s, T);
    results[1] = JMT(start_d, end_d, T);
    coeffs = results;

	return results;
}






vector<string> vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    string state = this->state;
    if(state.compare("KL") == 0) {
        if (lane > 0 ) states.push_back("PLCL");
        if (lane < lanes_available-1) states.push_back("PLCR");
        states.push_back("KL");
    } else if (state.compare("PLCL") == 0) {
            states.push_back("PLCL");
            states.push_back("KL");
    } else if (state.compare("PLCR") == 0) {
            states.push_back("PLCR");
            states.push_back("KL");
    }
	
    return states;
}



vector<vector<double> > vehicle::surroundings(vector<vector<double> > const &sensor_fusion) {
    int size = sensor_fusion.size();
    int future_t = 0;


    vector<vector<double> > results;
    for (int i = 0; i < size; ++i) {
        double car_d = sensor_fusion[i][6];
        if (car_d < 0) continue;

        double car_vx = sensor_fusion[i][3];
        double car_vy = sensor_fusion[i][4];
        double car_speed = sqrt(car_vx*car_vx + car_vy*car_vy);    
        double car_s = sensor_fusion[i][5];
        double delta_s = car_s - this->s;
        
        if (abs(delta_s) < PARAM_DETECT_RANGE) {
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




Point3 vehicle::keep_lane_trajectory(Point3 start_s) {
	double accel = PARAM_MAX_ACCEL;
	
    double ss = start_s.f;
    double vv = start_s.f_dot;
    double aa = start_s.f_ddot;

	double car_s = 0, car_v = 0, car_a = 0;
	if (!front_car_exist) {
        car_a = 0;
		car_v = target_speed;
		car_s = ss + T * 0.5 * (vv + car_v);	
	} else {
		double front_speed = front_car[0];
		double front_dist = front_car[1];
	    
        car_v = front_speed > target_speed ? target_speed : front_speed;
        car_a = 0;
        if (front_dist - T*0.5*(v+car_v) + front_speed*T < PARAM_DIST_SAFTY) {
            car_s = ss + front_dist + front_speed*T - PARAM_DIST_SAFTY;
        } else {
            car_s = ss + T*0.5*(vv+car_v);
        }
    }

    return Point3(car_s, car_v, car_a);
}





vector<Point3> vehicle::lane_change_trajectory(string st, Point3 start_s, Point3 start_d) {
    Point3 end_s, end_d;
    int final_lane = target_lane;

    end_d = Point3(final_lane*4.0+2.0, 0, 0);

    end_s.f = start_s.f + T * start_s.f_dot;
    end_s.f_dot = start_s.f_dot;
    end_s.f_ddot = 0;

    return {end_s, end_d};
}




double vehicle::prepare_lane_change_trajectory(string st, Point3 start_s, Point3 &end_s,  vector<vector<vector<double> > > & predictions) {
    int index = st == "PLCL" ? 0 : 1;
    double result_speed = 0;
    
    end_s = keep_lane_trajectory(start_s);

    if (predictions[index].size() == 0) {
        result_speed = PARAM_MAX_SPEED;
    } else if (predictions[index].size() == 1) {
        double delta_s = predictions[index][0][0];
        double car_speed = predictions[index][0][1];
        double future_delta_s = delta_s + car_speed*T - start_s.f_dot*T;
        if (delta_s >= PARAM_DIST_SAFTY && future_delta_s >= PARAM_DIST_MERGE) {
            result_speed = car_speed > target_speed ? target_speed : car_speed;
        } else if (delta_s <= -PARAM_DIST_SAFTY_BACK && future_delta_s < -PARAM_DIST_MERGE_BACK) {
            result_speed = target_speed;
        }
    } else {
        for (int i = 1; i < predictions[index].size(); ++i) {
            double delta_s1 = predictions[index][i-1][0];
            double car_speed1 = predictions[index][i-1][1];
            double delta_s2 = predictions[index][i][0];
            double car_speed2 = predictions[index][i][1];
            if (delta_s1 < 0 && delta_s2 > 0) {
                double future_delta_s1 = delta_s1 + (car_speed1-start_s.f_dot)*T;
                double future_delta_s2 = delta_s2 + (car_speed2-start_s.f_dot)*T;
                if (delta_s1 < -PARAM_DIST_SAFTY_BACK && delta_s2 > PARAM_DIST_SAFTY && future_delta_s1 < -PARAM_DIST_MERGE_BACK && future_delta_s2 > PARAM_DIST_MERGE) {
                    result_speed = car_speed2;
                }
            }
        }
    }
    return result_speed;
}







bool vehicle::check_collision(string st, vector<vector<vector<double> > > &predictions) {
    int index = st == "LCL" ? 0 : 1;
    
    bool result = false;
    for (int i = 0; i < predictions[index].size(); ++i) {
        double delta_s = predictions[index][i][0];
        double car_speed = predictions[index][i][1];
        if (delta_s < PARAM_DIST_MERGE && delta_s > -PARAM_DIST_MERGE_BACK) return true;
        double future_delta_s = (car_speed - v) * T + delta_s;
        if (future_delta_s < PARAM_DIST_MERGE && future_delta_s > -PARAM_DIST_MERGE_BACK) return true;
    }

    return result;
}




int vehicle::lane_determine(double car_d) {
    int result = -1;
    if (car_d >= 0 && car_d < 4) result = 0;
    else if (car_d >= 4 && car_d < 8) result = 1;
    else if (car_d >= 8 && car_d < 12) result = 2;
    
    return result;
}






vector<double> vehicle::JMT(Point3 &Start, Point3 &End, double T) {
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

    vector<double> start(3);
    start[0] = Start.f;
    start[1] = Start.f_dot;
    start[2] = Start.f_ddot;

    vector<double> end(3);
    end[0] = End.f;
    end[1] = End.f_dot;
    end[2] = End.f_ddot;

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




// evaluation of polynom
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
