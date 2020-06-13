#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helpers.h"

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // initialize with lane 1 and reference speed
  int lane = 1;
  double ref_vel = 0.0; // mph

  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                if (length && length > 2 && data[0] == '4' && data[1] == '2') {

                  auto s = hasData(data);

                  if (s != "") {
                    auto j = json::parse(s);

                    string event = j[0].get<string>();

                    if (event == "telemetry") {
                      // j[1] is the data JSON object

                      // Main car's localization Data
                      double car_x = j[1]["x"];
                      double car_y = j[1]["y"];
                      double car_s = j[1]["s"];
                      double car_d = j[1]["d"];
                      double car_yaw = j[1]["yaw"];
                      double car_speed = j[1]["speed"];

                      // Previous path data given to the Planner
                      auto previous_path_x = j[1]["previous_path_x"];
                      auto previous_path_y = j[1]["previous_path_y"];
                      // Previous path's end s and d values
                      double end_path_s = j[1]["end_path_s"];
                      double end_path_d = j[1]["end_path_d"];

                      // Sensor Fusion Data, a list of all other cars on the same side of the road.
                      auto sensor_fusion = j[1]["sensor_fusion"];

                      // Provided previous path point size.
                      int prev_size = previous_path_x.size();

                      // Preventing collitions.
                      if (prev_size > 0) {
                        car_s = end_path_s;
                      }

                      // find ther cars
                      bool car_ahead = false;
                      bool car_left = false;
                      bool car_right = false;

                      for ( int i = 0; i < sensor_fusion.size(); i++ ) 
                      {
                        float d = sensor_fusion[i][6];
                        int car_lane= -1;
                        // which lane other car at
                        if ( d > 0 && d < 4 ) {
                          car_lane = 0;
                        } else if ( d > 4 && d < 8 ) {
                          car_lane = 1;
                        } else if ( d > 8 && d < 12 ) {
                          car_lane = 2;
                        }
                        if (car_lane < 0) {
                          continue;
                        }

                        // speed of other car
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double near_car_speed = sqrt(vx*vx + vy*vy);
                        double near_car_s = sensor_fusion[i][5];
                        // danger region setting in frenet coodinate.
                        double danger_r = 28.0;
                        double r_offset = 15.0;

                        // predict other car's future position, 0.02 timestep
                        near_car_s += ((double)prev_size * 0.02 * near_car_speed);

                        // check if other car is in the danger region
                        if ( car_lane == lane ) {
                          car_ahead |= near_car_s > car_s && near_car_s - car_s <= danger_r;
                        } else if ( car_lane - lane == -1 ) {
                          car_left |= car_s - danger_r + r_offset <= near_car_s && car_s + danger_r >= near_car_s;
                        } else if ( car_lane - lane == 1 ) {
                          car_right |= car_s - danger_r + r_offset <= near_car_s && car_s + danger_r >= near_car_s;
                        }
                      }

                      // Behavior planner.
                      double vel_diff = 0.0;
                      const double MAX_VEL = 49.5;
                      const double MAX_ACC = .224;

                      if ( car_ahead ) {
                        // if no car left and I am not at lane 0.
                        if ( !car_left && lane > 0 ) {
                          lane = lane -1;
                        // if there is no car in right and ego vehicle is not in the lane 2.
                        } else if ( !car_right && lane != 2 ) {
                          lane = lane + 1;
                        } 
                        else {
                          vel_diff = -MAX_ACC;
                        }
                      } 
                      else {
                        // if I am not on the center lane
                        if ( lane != 1 ) {
                          if ( ( lane == 0 && !car_right) || ( lane == 2 && !car_left ) ) {
                            lane = 1; 
                          }
                        }
                        if ( ref_vel < MAX_VEL ) {
                          vel_diff = MAX_ACC;
                        }
                      }
                     
                      /**
                      * path generation
                      */

                      // template vectors
                      vector<double> ptsx;
                      vector<double> ptsy;
                      vector<double> next_x_vals;
                      vector<double> next_y_vals;

                      // ref_x is current position
                      double ref_x = car_x;
                      double ref_y = car_y;
                      double ref_yaw = deg2rad(car_yaw);

                      // use the car as starting point
                      if ( prev_size < 4 ) {

                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        //push back as ptsx in [prev_car_x1, car_x1, prev_car_x2, car_x2]
                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                      } else {
                        //Use the previous path's end points as starting reference
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];

                        //Use previous two points for calculating the reference yaw angle.
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        //Push the reference points to the ptsx,ptsy
                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                      }


                      vector<double> next_w0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                      vector<double> next_w1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                      vector<double> next_w2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                      // far point for fitting spline. 
                      ptsx.push_back(next_w0[0]);
                      ptsx.push_back(next_w1[0]);
                      ptsx.push_back(next_w2[0]);

                      ptsy.push_back(next_w0[1]);
                      ptsy.push_back(next_w1[1]);
                      ptsy.push_back(next_w2[1]);

                      // transfer coordinates to car's system
                      for ( int i = 0; i < ptsx.size(); i++ ) {
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;
                        ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                        ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
                      }

                      // Create the spline.
                      tk::spline s;
                      // Set (x,y) points to the spline [x1,x2,x3...],[y1,y2,y3...]
                      s.set_points(ptsx, ptsy);// fitting the points

                      for ( int i = 0; i < prev_size; i++ ) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                      }

                      double target_x = 30.0;
                      double target_y = s(target_x);
                      double target_dist = sqrt(target_x*target_x + target_y*target_y);

                      double x_add_on = 0;
                      // limit the jerk
                      for( int i = 0; i <= 50 - prev_size; i++ ) {
                        ref_vel += vel_diff;

                        if ( ref_vel >= MAX_VEL ) {
                          ref_vel = MAX_VEL - 0.1;
                        } else if ( ref_vel <= MAX_ACC ) {
                          ref_vel += MAX_ACC;
                        }

                        //generates 50 ponits
                        double N = target_dist/(0.02*ref_vel/2.24);//2.24 is distance between ref_point to target with the given velocity
                        double x_point = x_add_on + target_x/N;
                        double y_point = s(x_point);
                        double x_ref = x_point;
                        double y_ref = y_point;
                        x_add_on = x_point;
                        // rotate points
                        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
                        // translate points
                        x_point += ref_x;
                        y_point += ref_y;
                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                      }


                      json msgJson;

                      msgJson["next_x"] = next_x_vals;
                      msgJson["next_y"] = next_y_vals;

                      auto msg = "42[\"control\","+ msgJson.dump()+"]";

                      //this_thread::sleep_for(chrono::milliseconds(1000));
                      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    }
                  } else {
                    // Manual driving
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                  }
                }
              });

  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}