#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

#define MAX_SPEED 49.5
#define SAFETY_DISTANCE 35.0


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  double ref_velocity_mph = 0.0; // Reference velocity
  int lane_num = 1; // default lane

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ref_velocity_mph,&lane_num]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          int prev_path_size = previous_path_x.size();

            if(prev_path_size  > 0)
            {
              car_s = end_path_s;
            }
          
          // SECTION 1: safety flag setting
          // Lane shift flag setting according to sensor fusion data to avoid collision and maintaining the speed as possible.
          // collision warning flag and Lane availability flags
         double ahead_car_speed;
         bool collision_warning = false;
         bool left_shift_safe = true;
         bool right_shift_safe = true;

         for (int i = 0; i  < sensor_fusion.size(); ++i)
          {

              float d = sensor_fusion[i][6];            
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];              

              // check_car_s in the future
              check_car_s += (double)prev_path_size*0.02*check_speed; 
              
              // if the gap of main car with reference car is less than safe distance
              if ( ((check_car_s -  car_s) > -25.0) || ( check_car_s > car_s ) ) && ( ( check_car_s - car_s) < SAFETY_DISTANCE ) ) 
              {

                // collision warning flag
                if(( d < (2 + 4* lane_num + 2)) && (d > (2 + 4*lane_num - 2)) && (check_car_s > car_s))
                {
                collision_warning = true;
                ahead_car_speed = check_speed * 2.24; 
                }
                // We know that left most lane number is 0 and right most lane number is 2

                // left lane shift safety flag
                int left_shift_lane = lane_num - 1;
                if( lane_num == 0 || ( ( d < (2 + 4*(left_shift_lane) + 2) ) && ( d > (2 + 4*(left_shift_lane) - 2) ) ) )
                {
                  left_shift_safe = false;
                }

                // right lane shift safety flag
                int right_shift_lane = lane_num + 1;
                if( lane_num == 2 || ( ( d < (2 + 4*(right_shift_lane) + 2) ) && ( d > (2 + 4*(right_shift_lane) - 2) ) ) )
                {
                  right_shift_safe = false;
                }

              } 

            }


          //  SECTION 2: Lane shift

          if( collision_warning )
          {

            // shift to left lane if we are not in the left most lane and left lane is safe to shift  
            if(left_shift_safe ) {lane_num-=1;}  
            // shift to right lane if we are not in the right most lane and right lane is safe to shift            
            else if(right_shift_safe)  {lane_num+=1;}
            // if no lane is available, follow the front car and decelerate(with the given limit) to avoid collision.
            else{ if( ahead_car_speed < MAX_SPEED ){ref_velocity_mph -= 0.224;} }

          }

          // if there is no car in the front accelerate upto the maximum speed limit
          else if(ref_velocity_mph < MAX_SPEED) { ref_velocity_mph += 0.224;} 

          //  SECTION 3: Path decide

          vector<double> ptsy;
          vector<double> ptsx;

          // Calculate reference x, reference y, reference yaw
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          
          if (prev_path_size < 2)
          {

            // action for previous size is almost empty
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsy.push_back(prev_car_y);
            ptsx.push_back(car_x);
            ptsy.push_back(car_y);

          }
          else
          {

            // action for previous size bigger than 1
            ref_x = previous_path_x[prev_path_size-1];
            ref_y = previous_path_y[prev_path_size-1];

            double ref_x_prev = previous_path_x[prev_path_size - 2];
            double ref_y_prev = previous_path_y[prev_path_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            ptsx.push_back(ref_x_prev);
            ptsy.push_back(ref_y_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);

          }

          // SECTION 4 : spaced points == > Referred from Project Q & A

          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane_num), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane_num), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane_num), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Shift
          for(int i =0; i < ptsx.size(); ++i)
          {

            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));

          }

          // SECTION 5 : spline for trajectory

          // create a spline
          tk::spline s;

          // set (x, y) points to the spline ==> referred from Path Planning Walkthrough ( Project Q & A video )
          s.set_points(ptsx, ptsy);

          // start with all of the previous path points from last time
          for(int i = 0; i < previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate how to break up spline points so that we travel at our desired reference velocity

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt( target_x * target_x + target_y * target_y);
          double x_add_on = 0.0;

          // // Breaking the target distance into N points
          // double N = target_dist / (0.02*ref_velocity_mph/2.24);
          
          // Fill up the rest of our path planner after filling it with prev
          for (int i = 0; i <= 50 - previous_path_x.size(); ++i)
          {

            // Breaking the target distance into N points
            double N = target_dist / ( 0.02*ref_velocity_mph / 2.24 );
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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