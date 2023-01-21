/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 				Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 **/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <iostream>
#include <fstream>
#include <typeinfo>

#include "json.hpp"
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include "Eigen/QR"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"
#include "pid_controller.h"
#include "stanley_controller.h"

#include <limits>
#include <iostream>
#include <fstream>
#include <uWS/uWS.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <time.h>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES


string hasData(string s) {

  auto found_null = s.find("null");
    auto b1 = s.find_first_of("{");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
      return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 1);
    }
    return "";

}

template <typename T> int sgn(T val) {

    return (T(0) < val) - (val < T(0));

}

double angle_between_points(double x1, double y1, double x2, double y2){

  return atan2(y2-y1, x2-x1);

}

BehaviorPlannerFSM behavior_planner(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Decalre and initialized the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;

void path_planner(
  vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, 
  double yaw, double velocity, State goal, bool is_junction, string tl_state, 
  vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, 
  vector< vector<double> >& spirals_v, vector<int>& best_spirals
) {

  State ego_state;

  ego_state.location.x = x_points[x_points.size()-1];
  ego_state.location.y = y_points[y_points.size()-1];
  ego_state.velocity.x = velocity;

  if (x_points.size() > 1) {
  	ego_state.rotation.yaw = angle_between_points(
      x_points[x_points.size()-2], y_points[y_points.size()-2], x_points[x_points.size()-1], y_points[y_points.size()-1]
    );
  	ego_state.velocity.x = v_points[v_points.size()-1];
  	if (velocity < 0.01)
  		ego_state.rotation.yaw = yaw;
  }

  Maneuver behavior = behavior_planner.get_active_maneuver();

  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  if (behavior == STOPPED) {
  	unsigned int max_points = 20;
  	double point_x = x_points[x_points.size()-1];
  	double point_y = y_points[x_points.size()-1];
  	while (x_points.size() < max_points) {
  	  x_points.push_back(point_x);
  	  y_points.push_back(point_y);
  	  v_points.push_back(0);
  	}
  	return;
  }

  auto goal_set = motion_planner.generate_offset_goals(goal);

  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);

  auto desired_speed = utils::magnitude(goal.velocity);

  State lead_car_state;  // = to the vehicle ahead...

  if (spirals.size() == 0) {
  	cout << "Error: No spirals generated " << endl;
  	return;
  }

  for (unsigned int i = 0; i < spirals.size(); i++) {

    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory(
      spirals[i], desired_speed, ego_state, lead_car_state, behavior);

    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;
    for (unsigned int j = 0; j < trajectory.size(); j++) {
      double point_x = trajectory[j].path_point.x;
      double point_y = trajectory[j].path_point.y;
      double velocity = trajectory[j].v;
      spiral_x.push_back(point_x);
      spiral_y.push_back(point_y);
      spiral_v.push_back(velocity);
    }

    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);
  }

  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = -1;

  if (best_spirals.size() > 0)
  	best_spiral_idx = best_spirals[best_spirals.size()-1];

  int index = 0;
  unsigned int max_points = 20;
  int add_points = spirals_x[best_spiral_idx].size();
  while (x_points.size() < max_points && index < add_points) {
    double point_x = spirals_x[best_spiral_idx][index];
    double point_y = spirals_y[best_spiral_idx][index];
    double velocity = spirals_v[best_spiral_idx][index];
    index++;
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(velocity);
  }

}

void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag) {

	for (unsigned int i = 0; i < x_points.size(); i++) {
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
		obstacles.push_back(obstacle);
	}
	obst_flag = true;

}

int main ()
{
  cout << "starting server" << endl;
  uWS::Hub h;

  double new_delta_time;
  int i = 0;

  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();

  time_t prev_timer;
  time_t timer;
  time(&prev_timer);

  /**
  * Step 1: Create and initialize lateral controller (steering controller)
  **/  
  // Create PID controller instance for lateral control based on crosstrack error only
  PID pid_steer = PID();
  // Initialize lateral PID controller
  double FF_steer = 0.0;
  double Kp_steer = 0.3;
  double Ki_steer = 0.001;
  double Kd_steer = 0.3;
  double output_lim_min_steer = -1.2;
  double output_lim_max_steer = 1.2;
  double int_errot_0_steer = 0.0;
  pid_steer.Init(FF_steer, Kp_steer, Ki_steer, Kd_steer, output_lim_max_steer, output_lim_min_steer, int_errot_0_steer);

  // Create Stanley controller instance as alternative for lateral control based on both crosstrack and heading error
  STANLEY stanley_steer = STANLEY();
  // Initialize lateral Stanley controller
  double K_crosstrack = 1.0;
  double velocity_threshold = 1e-3;
  stanley_steer.Init(K_crosstrack, velocity_threshold, output_lim_max_steer, output_lim_min_steer);

  /**
  * Step 1: create pid (pid_throttle) for throttle command and initialize values
  **/
  // Create longitudinal PID controller
  PID pid_throttle = PID();
  // Initialize longitudinal PID controller
  double FF_throttle = 0.0;
  double Kp_throttle = 0.2;
  double Ki_throttle = 0.0009;
  double Kd_throttle = 0.1;
  double output_lim_min_throttle = -1.0;
  double output_lim_max_throttle = 1.0;
  double int_error_0_throttle = 0.0;
  pid_throttle.Init(FF_throttle, Kp_throttle, Ki_throttle, Kd_throttle, output_lim_max_throttle, output_lim_min_throttle, int_error_0_throttle);

  // Execute control cycle on receiving a new message from SimulationAPI
  h.onMessage([&pid_steer, &stanley_steer, &pid_throttle, &new_delta_time, &timer, &prev_timer, &i, &prev_timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
        // Check if current message has data
        auto s = hasData(data);

        if (s != "") {

          // Parse data from current message
          auto data = json::parse(s);

          // Create file to save values
          fstream file_steer;
          file_steer.open("steer_pid_data.txt");
          fstream file_throttle;
          file_throttle.open("throttle_pid_data.txt");

          // Planned trajectory points (interpolated waypoints)
          vector<double> x_points = data["traj_x"];
          vector<double> y_points = data["traj_y"];
          vector<double> v_points = data["traj_v"];
          // Actual yaw angle of the ego vehicle pose
          double yaw = data["yaw"];
          // Actual velocity of the ego vehicle
          double velocity = data["velocity"];
          // Simulation time
          double sim_time = data["time"];
          // Planned waypoints
          double waypoint_x = data["waypoint_x"];
          double waypoint_y = data["waypoint_y"];
          double waypoint_t = data["waypoint_t"];
          bool is_junction = data["waypoint_j"];
          // Traffic light state
          string tl_state = data["tl_state"];
          // Actual position of the ego vehicle (center point)
          double x_position = data["location_x"];
          double y_position = data["location_y"];
          double z_position = data["location_z"];

          // Obstacle positions 
          if (!have_obst) {
          	vector<double> x_obst = data["obst_x"];
          	vector<double> y_obst = data["obst_y"];
          	set_obst(x_obst, y_obst, obstacles, have_obst);
          }

          // Goal states derived from the planned waypoints
          State goal;
          goal.location.x = waypoint_x;
          goal.location.y = waypoint_y;
          goal.rotation.yaw = waypoint_t;

          // Define vectors for spline interpolation of the planned trajectory given by the waypoints
          vector< vector<double> > spirals_x;
          vector< vector<double> > spirals_y;
          vector< vector<double> > spirals_v;
          vector<int> best_spirals;

          // Path planning using spiral interpolation of the planned waypoints
          path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);

          // Save time and compute delta time
          time(&timer);
          new_delta_time = difftime(timer, prev_timer);
          prev_timer = timer;

          //////////////////////////////////////////
          // Lateral control (steeroing control)
          //////////////////////////////////////////

          // Choose between PID (default: USE_PID = true) and Stanley lateral control (alternative)
          bool USE_PID_LAT = true;

          /**
          * Step 2: Compute crosstrack_error and headding_error from desired trajectory and the current ego vehicle pose
          **/
          // Update the delta time with the previous command
          pid_steer.UpdateDeltaTime(new_delta_time);

          // Calculate center point of the front axle position
          double ego_wheel_base = 4.0;
          double x_position_front = x_position + ego_wheel_base/2 * cos(yaw);
          double y_position_front = y_position + ego_wheel_base/2 * sin(yaw);

          // Find the point of the planned path segment closest to the front axle center position of the ego vehicle (player)
          double crosstrack_error = 0;
          double heading_error = 0;
          double min_dist = DBL_MAX;  // Minimum distance of the planned path cto the ego vehicles front axle position
          unsigned int idx_min_dist = 0;  // Index for cross-track-error calculation
          for (unsigned int idx=0; idx<x_points.size()-1; ++idx) { // Leave out last element of the path segment from the search
            // Calculate the distance of the current point on the planned path segment to the ego vehicle front axle position
            double dist = sqrt(
              pow(x_points[idx] - x_position_front, 2) + pow(y_points[idx] - y_position_front, 2)
            );
            if (dist < min_dist) {
              // Update minimum distance (absolute value)
              min_dist = abs(dist);
              // Update index of the waypoint closest to the current ego vehicle position
              idx_min_dist = idx;
            }
          }

          // Calculate crosstrack_error as the distance of the ego vehicle's front axle center position
          // to the tangent through the closest point approximated and its successor on the planned
          // trajectory segment.
          // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
          // Ego vehicle front axle position is left of the path tangent => crosstrack_error < 0
          // Ego vehicle front axle position is right of the path tangent => crosstrack_error > 0
          // Ego vehicle front axcle position is on the path tangent => crosstrack_error == 0
          crosstrack_error = (
            (
              (x_points[idx_min_dist+1] - x_points[idx_min_dist]) * (y_points[idx_min_dist] - y_position_front) -
              (y_points[idx_min_dist+1] - y_points[idx_min_dist]) * (x_points[idx_min_dist] - x_position_front)
            ) / sqrt(
              pow(x_points[idx_min_dist+1] - x_points[idx_min_dist], 2) +
              pow(y_points[idx_min_dist+1] + y_points[idx_min_dist], 2)
            )
          )
          // Calculate yaw angle of the planned path segment at the waypoint closest to the current ego vehicle position
          double yaw_path = angle_between_points(
            x_points[idx_min_dist], y_points[idx_min_dist], x_points[idx_min_dist+1], y_points[idx_min_dist+1]
          );
          // Compute heading_error
          heading_error = yaw_path - yaw;
          // Init steer correction output (considering both heading_error and crosstrack_error)
          double steer_output;

          /**
          * Step 3: Compute lateral control command
          **/
          // Compute steer control command to apply
          if (USE_PID_LAT) {            
            // Use PID controller
            pid_steer.UpdateError(crosstrack_error);
            steer_output = pid_steer.GetControlCommand();
          } else {
            // Use Stanley controller
            steer_output = stanley_steer.GetSteerCommand(heading_error, crosstrack_error, velocity);
          }

          // Save lateral control data
          file_steer.seekg(std::ios::beg);
          for (int j=0; j < i - 1; ++j) {
              file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          }
          file_steer  << i ;
          file_steer  << " " << heading_error;
          file_steer  << " " << crosstrack_error;
          file_steer  << " " << steer_output << endl;
          file_throttle.seekg(std::ios::beg);

          //////////////////////////////////////////
          // Longidudinal control (throttle control)
          //////////////////////////////////////////
          
          /**
          * Step 2: Compute velocity_error from desired velocity and actual velocity
          **/
          // Update the delta time with the previous command
          pid_throttle.UpdateDeltaTime(new_delta_time);

          // Compute velocity_error
          double velocity_error = v_points[idx_min_dist] - velocity;
          double throttle_output;
          double brake_output;

          /**
          * Step 3: Compute longitudinal control command
          **/
          // Compute longidudinal control command to apply
          pid_throttle.UpdateError(velocity_error);
          double throttle = pid_throttle.GetControlCommand();

          // Adapt the negative throttle to break
          if (throttle > 0.0) {
            throttle_output = throttle;
            brake_output = 0;
          } else {
            throttle_output = 0;
            brake_output = -throttle;
          }

          // Save longitudinal control data
          file_throttle.seekg(std::ios::beg);
          for (int j=0; j < i - 1; ++j) {
              file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
          }
          file_throttle  << i ;
          file_throttle  << " " << velocity_error;
          file_throttle  << " " << throttle_output;
          file_throttle  << " " << brake_output << endl;

          // Send control signal
          json msgJson;
          msgJson["brake"] = brake_output;
          msgJson["throttle"] = throttle_output;
          msgJson["steer"] = steer_output;

          // Sent planned path and maneuver data
          msgJson["trajectory_x"] = x_points;
          msgJson["trajectory_y"] = y_points;
          msgJson["trajectory_v"] = v_points;
          msgJson["spirals_x"] = spirals_x;
          msgJson["spirals_y"] = spirals_y;
          msgJson["spirals_v"] = spirals_v;
          msgJson["spiral_idx"] = best_spirals;
          msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();

          // Minimum point threshold before doing the update
          // for high update rate use 19 for slow update rate use 4
          msgJson["update_point_thresh"] = 16;

          auto msg = msgJson.dump();

          i = i + 1;
          file_steer.close();
          file_throttle.close();

      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

    }

  });


  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
      cout << "Connected!!!" << endl;
    });


  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
      ws.close();
      cout << "Disconnected" << endl;
    });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
    {
      cout << "Listening to port " << port << endl;
      h.run();
    }
  else
    {
      cerr << "Failed to listen to port" << endl;
      return -1;
    }

}
