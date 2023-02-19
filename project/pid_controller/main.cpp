/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *     Authors: Munir Jojo-Verge
 *              Aaron Brown
 * Modified by: Andreas Albrecht
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
    } else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}


// Template of a signmum function
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


// Calculate angle between x-axis (cartesian coordinates) and the line through points [x1, y1] and [x2, y2]
double angle_between_points(double x1, double y1, double x2, double y2) {
  return atan2(y2-y1, x2-x1);
}


// Calculate distance between points in 2D space
double distance_between_points(double x1, double y1, double x2, double y2) {
  return sqrt(pow(y2-y1, 2) + pow(x2-x1, 2));
}


// Declare and initialize Behavior Planner and all its class requirements
BehaviorPlannerFSM behavior_planner(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Declare and initialized the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);


// Define vector for obstacle positions plus obstacle flag
bool have_obst = false;
vector<State> obstacles;


// Path segment planner
void path_planner(
  vector<double>& x_points, vector<double>& y_points, vector<double>& v_points,
  double yaw, double velocity,
  State goal, bool is_junction, string tl_state,
  vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y,
  vector< vector<double> >& spirals_v, vector<int>& best_spirals
) {

  State ego_state;

  // Initialize ego vehicle position and velocity based on last point on the driven trajectory
  ego_state.location.x = x_points[x_points.size()-1];
  ego_state.location.y = y_points[y_points.size()-1];
  //ego_state.location.x = x_position;
  //ego_state.location.y = y_position;
  ego_state.velocity.x = velocity;

  // Estimatate current ego vehicle yaw angle
  if ( x_points.size() > 1 ) {
  	ego_state.rotation.yaw = angle_between_points(
      x_points[x_points.size()-2], y_points[y_points.size()-2],
      x_points[x_points.size()-1], y_points[y_points.size()-1]
    );
  	ego_state.velocity.x = v_points[v_points.size()-1];
  	if(velocity < 0.01)
  		ego_state.rotation.yaw = yaw;
  }
  // Get current ego vehicle heading (yaw angle)
  //ego_state.rotation.yaw = yaw;

  // Get new maneuver from behavior planner
  Maneuver behavior = behavior_planner.get_active_maneuver();

  // Set goal point based on state transition for the new maneuver
  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  // Set target speed to zero on the final path segment if vehicle is supposed to stop
  if ( behavior == STOPPED ) {
  	unsigned int max_points = 20;
  	double point_x = x_points[x_points.size()-1];
  	double point_y = y_points[x_points.size()-1];
  	while ( x_points.size() < max_points ) {
  	  x_points.push_back(point_x);
  	  y_points.push_back(point_y);
  	  v_points.push_back(0);
  	}
  	return;
  }

  // Generate offset goal state
  auto goal_set = motion_planner.generate_offset_goals(goal);

  // Generate spiral path segments between ego state and goal state
  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);

  // Desired goal speed
  auto desired_speed = utils::magnitude(goal.velocity);

  State lead_car_state;  // = to the vehicle ahead...

  if ( spirals.size() == 0 ) {
  	cout << "Error: No spirals generated " << endl;
  	return;
  }

  // Generate velocity profile for all spiral trajectories
  for ( unsigned int i = 0; i < spirals.size(); i++ ) {
    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory(
      spirals[i], desired_speed, ego_state, lead_car_state, behavior
    );
    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;
    for ( unsigned int j = 0; j < trajectory.size(); j++ ) {
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

  // Get the best spiral trajectory avoiding collisions with obstacles
  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = -1;

  if ( best_spirals.size() > 0 ) 
  	best_spiral_idx = best_spirals[best_spirals.size()-1];

  // Generate waypoints for next path segment based on best trajectory
  unsigned int index = 0;
  unsigned int max_points = 20;
  unsigned int add_points = spirals_x[best_spiral_idx].size();
  while ( x_points.size() < max_points && index < add_points ) {
    double point_x = spirals_x[best_spiral_idx][index];
    double point_y = spirals_y[best_spiral_idx][index];
    double velocity = spirals_v[best_spiral_idx][index];
    index++;
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(velocity);
  }

}


// Set obstacle positions
void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag) {

	for ( unsigned int i = 0; i < x_points.size(); i++ ) {
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
		obstacles.push_back(obstacle);
	}
	obst_flag = true;

}


// Transform planned path segments into the ego vehicle's coordinate system
void transform_path_to_ego_coordinates(
  vector<double>& x_points_ego, vector<double>& y_points_ego,
  vector<double>& x_points, vector<double>& y_points,
  double x_position, double y_position, double yaw
) {
  // Simple 2D backtransformation from absolute into relative coordinates
  for ( unsigned int i = 0; i < x_points.size(); i++ ) {
    double u_abs = x_points[i] - x_position;
    double v_abs = y_points[i] - y_position;
    double u_rel = u_abs * cos(yaw) + v_abs * sin(yaw);
    double v_rel = u_abs * (-sin(yaw)) + v_abs * cos(yaw);
    x_points_ego.push_back(u_rel);
    y_points_ego.push_back(v_rel);
  }
}


// Find closest waypoint on path segment in driving direction using ego coordinates
unsigned int find_next_waypoint_in_ego_coordinates(vector<double>& x_path_ego, vector<double>& y_path_ego) {
  unsigned int idx_min_dist = 0;
  double min_dist = DBL_MAX;
  // Loop over all path segment points with a positive x-coordinate
  for ( unsigned int idx=0; idx<x_path_ego.size(); ++idx ) {
    if ( x_path_ego[idx] > 0 ) {
      // Calculate the distance of the current point on the path segment to the reference point
      double dist = distance_between_points(x_path_ego[idx], y_path_ego[idx], 0.0, 0.0);
      if (dist < min_dist) {
        // Update minimum distance (absolute value)
        min_dist = dist;
        // Update index of the waypoint closest to the reference point
        idx_min_dist = idx;
      }
    }
  }
  return idx_min_dist;
}


// Find closest point on a path segment to a given reference position using absolute coordinates
unsigned int find_closest_point_on_path_segment(vector<double>& x_path, vector<double>& y_path, double x_ref, double y_ref) {
  unsigned int idx_min_dist = 0;
  double min_dist = DBL_MAX;
  // Loop over all path segment points
  for ( unsigned int idx=0; idx<x_path.size(); ++idx ) {
    // Calculate the distance of the current point on the path segment to the reference point
    double dist = distance_between_points(x_path[idx], y_path[idx], x_ref, y_ref);
    if (dist < min_dist) {
      // Update minimum distance (absolute value)
      min_dist = dist;
      // Update index of the waypoint closest to the reference point
      idx_min_dist = idx;
    }
  }
  return idx_min_dist;
}


int main ()
{
  // Start uWebSocket Server
  cout << "starting server" << endl;
  uWS::Hub h;

  // Init simulation step counter
  int i = 0;

  // Create log files to save the pid_controller values
  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();

  // Define timer
  time_t prev_timer;
  time_t timer;
  time(&prev_timer);

  // Define delta time
  double new_delta_time;

  /**
  * (Step 1): Create PID (pid_steer) steer controller for longitudinal motion control
  **/
  PID pid_steer = PID();
  // Set PID steer control parameters
  double Kp_steer = 0.18; // 0.21; // 0.3;
  double Ki_steer = 0.008; // 0.001; // 0.01;
  double Kd_steer = 0.28; // 0.1; // 0.3; // 0.6;
  double output_lim_min_steer = -1.2;
  double output_lim_max_steer = 1.2;
  double int_win_min_steer = -0.6;
  double int_win_max_steer = 0.6;
  double delta_t_min_steer = 1.0e-3;
  double initial_steer_error = 0.0;
  // Initialize PID steer controller for lateral motion control
  pid_steer.Init(
    Kp_steer,
    Ki_steer,
    Kd_steer,
    output_lim_min_steer,
    output_lim_max_steer,
    int_win_min_steer,
    int_win_max_steer,
    delta_t_min_steer,
    initial_steer_error
  );

  /**
  * (Step 1): Create PID (pid_throttle) for throttle control command and initialize values
  **/
  PID pid_throttle = PID();
  // Set PID throttle control parameters
  double Kp_throttle = 0.17; // 0.5; // 0.3 // 0.25;
  double Ki_throttle = 0.004; // 0.02; // 0.001; // 0.0009;
  double Kd_throttle = 0.0; // 0.08; // 0.0; // 0.1;
  double output_lim_min_throttle = -1.0;
  double output_lim_max_throttle = 1.0;
  double int_win_min_throttle = -0.5;
  double int_win_max_throttle = 0.5;
  double delta_t_min_throttle = 1e-3;
  double intial_throttle_error = 0.0;
  // Initialize PID throttle controller for longitudinal motion control
  pid_throttle.Init(
    Kp_throttle,
    Ki_throttle,
    Kd_throttle,
    output_lim_min_throttle,
    output_lim_max_throttle,
    int_win_min_throttle,
    int_win_max_throttle,
    delta_t_min_throttle,
    intial_throttle_error
  );

  // Execute control cycle on receiving a new message from SimulationAPI
  h.onMessage(
    [&pid_steer, &pid_throttle, &new_delta_time, &timer, &prev_timer, &i, &prev_timer]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

      // Check if current message has data
      auto s = hasData(data);

      if ( s != "" ) {

        // Parse data from current message
        auto data = json::parse(s);

        // Open log files to save the pid_controller values
        fstream file_steer;
        file_steer.open("steer_pid_data.txt");
        fstream file_throttle;
        file_throttle.open("throttle_pid_data.txt");

        // Planned trajectory points (interpolated waypoints)
        vector<double> x_points = data["traj_x"];
        vector<double> y_points = data["traj_y"];
        vector<double> v_points = data["traj_v"];
        // Previous yaw angle of the ego vehicle
        double yaw = data["yaw"];
        // Actual velocity of the ego vehicle
        double velocity = data["velocity"];
        // Actual simulation time
        double sim_time = data["time"];
        // Next planned waypoint
        double waypoint_x = data["waypoint_x"];
        double waypoint_y = data["waypoint_y"];
        double waypoint_t = data["waypoint_t"];
        // Flag to mark junctions on the planned path
        bool is_junction = data["waypoint_j"];
        // Traffic light state
        string tl_state = data["tl_state"];
        // Actual position of the ego vehicle (center point)
        double x_position = data["location_x"];
        double y_position = data["location_y"];
        double z_position = data["location_z"];

        // Obstacle positions 
        if ( !have_obst ) {
          vector<double> x_obst = data["obst_x"];
          vector<double> y_obst = data["obst_y"];
          set_obst(x_obst, y_obst, obstacles, have_obst);
        }

        // Set goal state based on the next planned waypoint
        State goal;
        goal.location.x = waypoint_x;
        goal.location.y = waypoint_y;
        goal.rotation.yaw = waypoint_t;

        // Define vectors for the spiral trajectory variants to the next target waypoint
        vector< vector<double> > spirals_x;
        vector< vector<double> > spirals_y;
        vector< vector<double> > spirals_v;
        vector<int> best_spirals;

        // Plan next path segment using spiral interpolation to the next target waypoint
        path_planner(
          x_points, y_points, v_points,
          yaw, velocity,
          goal, is_junction, tl_state,
          spirals_x, spirals_y, spirals_v, best_spirals
        );

        // Define vectors for path segment transformed into ego vehicle coordinates
        vector<double> x_points_ego;
        vector<double> y_points_ego;

        // Convert the planned path segment into ego vehicle coordinates
        transform_path_to_ego_coordinates(x_points_ego, y_points_ego, x_points, y_points, x_position, y_position, yaw);

        // Save time and compute delta time
        time(&timer);
        new_delta_time = difftime(timer, prev_timer);
        prev_timer = timer;

        // Find closest waypoint on the planned path segment in driving direction (using ego vehicle coordinates)
        unsigned int closest_wp_idx = find_next_waypoint_in_ego_coordinates(x_points_ego, y_points_ego);
        // unsigned int closest_wp_idx = find_closest_point_on_path_segment(x_points, y_points, x_position, y_position);

        // Calculate distance to the closest point on the planned path segment
        double dist_closest_wp = distance_between_points(
          x_position, y_position, x_points[closest_wp_idx], y_points[closest_wp_idx]
        );

        // Define lookahead waypoint on the planned trajectory to get setpoints for lateral and longitudinal control
        unsigned int lookahead_wp_idx = x_points_ego.size() - 1;
        if (closest_wp_idx + 2 < x_points_ego.size()) {
          unsigned int lookahead_wp_idx = closest_wp_idx + 2;
        }
        
        // Calculate distance to the lookahead waypoint on the planned path segment
        double dist_lookahead_wp = distance_between_points(
          x_position, y_position, x_points[lookahead_wp_idx], y_points[lookahead_wp_idx]
        );

        ////////////////////////////////////////////////////
        // Lateral motion control (steering control)
        ////////////////////////////////////////////////////

        /**
          * Step 2: Get a desired heading from the current ego vehicle position towards the target waypoint on
          *         the planned trajectory to calculte a new setpoint for steering control
          **/
        // Calculate desired yaw angle from the current position to the closest waypoint on the planned trajectory
        double yaw_closest_wp = angle_between_points(
          x_position, y_position, x_points[closest_wp_idx], y_points[closest_wp_idx]
        );

        // Calculate heading error w.r.t. the closest waypoint on the planned trajectory
        double heading_error_closest_wp = yaw_closest_wp - yaw;

        // Calculate desired yaw angle from the current position to the lookahead waypoint on the planned trajectory
        double yaw_lookahead_wp = angle_between_points(
          x_position, y_position, x_points[lookahead_wp_idx], y_points[lookahead_wp_idx]
        );

        // Calculate heading error w.r.t. the lookahead waypoint on the planned trajectory
        double heading_error_lookahead_wp = yaw_lookahead_wp - yaw;

        // Get crosstrack error w.r.t. to the lookahed waypoint on the planned trajectory
        double cte_lookahead_wp = y_points_ego[lookahead_wp_idx];

        // Get crosstrack error w.r.t. to the closest waypoint on the planned trajectory
        double cte_closest_wp = y_points_ego[closest_wp_idx];

        /**
        * Step 3): Update steering control given the current cross-track error / or heading error
        **/
        // Define vector to hold the pid steer control errors and error gains
        vector<double> pid_steer_errors;
        vector<double> pid_steer_error_gains;

        // Define set point for steering control variable (select control variable)
        //double steer_setpoint = yaw_lookahead_wp;
        //double steer_setpoint = yaw_closest_wp;
        //double steer_setpoint = cte_lookahead_wp;
        double steer_setpoint = cte_closest_wp;

        // Define actual steering control variable (select control variable)
        double steer_act_value = 0;  // actual ego position is always zero in ego coordinates
        // double steer_act_value = yaw;

        // Update the delta time with the previous command
        pid_steer.UpdateDeltaTime(new_delta_time);

        // Compute PID steer control command to apply
        pid_steer.Update(steer_setpoint, steer_act_value);

        // Get PID steer control command
        double steer_output = pid_steer.GetControlCommand();

        // Get PID steer control errors and error gains
        pid_steer_errors = pid_steer.GetErrors();
        pid_steer_error_gains = pid_steer.GetErrorGains();

        // Decompose pid steer error vector
        double prop_steer_error = pid_steer_errors[0];
        double int_steer_error = pid_steer_errors[1];
        double diff_steer_error = pid_steer_errors[2];

        // Decompose pid steer error gain vector
        double prop_steer_error_gain = pid_steer_error_gains[0];
        double int_steer_error_gain = pid_steer_error_gains[1];
        double diff_steer_error_gain = pid_steer_error_gains[2];

        // Save lateral control data
        file_steer.seekg(std::ios::beg);
        for (int j=0; j < i - 1; ++j) {
          file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        file_steer << i ;
        file_steer << " " << steer_setpoint;
        file_steer << " " << steer_act_value;
        file_steer << " " << prop_steer_error;
        file_steer << " " << int_steer_error;
        file_steer << " " << diff_steer_error;
        file_steer << " " << prop_steer_error_gain;
        file_steer << " " << int_steer_error_gain;
        file_steer << " " << diff_steer_error_gain;
        file_steer << " " << steer_output;
        file_steer << " " << yaw_closest_wp;
        file_steer << " " << yaw_lookahead_wp;
        file_steer << " " << yaw;
        file_steer << " " << heading_error_closest_wp;
        file_steer << " " << heading_error_lookahead_wp;
        file_steer << " " << cte_closest_wp;
        file_steer << " " << cte_lookahead_wp;
        file_steer << " " << x_position;
        file_steer << " " << y_position;
        file_steer << " " << x_points[closest_wp_idx];
        file_steer << " " << y_points[closest_wp_idx];
        file_steer << " " << x_points[lookahead_wp_idx];
        file_steer << " " << y_points[lookahead_wp_idx];
        file_steer << " " << dist_closest_wp;
        file_steer << " " << dist_lookahead_wp << endl;

        ////////////////////////////////////////////////////
        // Longidudinal motion control (throttle control)
        ////////////////////////////////////////////////////
        /**
          * Step 2: Get a desired velocity setpoint from the target waypoint on the planned trajectory
          *         to calculte a new setpoint for throttle control
          **/
        // Get the desired velocity at the closest waypoint on the planned trajectory
        double velocity_closest_wp = v_points[closest_wp_idx];

        // Get the desired velocity at the lookahead waypoint on the planned trajectory
        double velocity_lookahead_wp = v_points[lookahead_wp_idx];

        /**
        * Step 3): Update throttle control given the current velocity and the desired velocity on the planned trajectory
        **/
        // Define vector to hold the pid throttle control errors and error gains
        vector<double> pid_throttle_errors;
        vector<double> pid_throttle_error_gains;

        // Define set point for velocity error calculation and throttle control input
        // double velocity_setpoint = velocity_lookahead_wp;
        double velocity_setpoint = velocity_closest_wp;

        // Update the delta time with the previous command
        pid_throttle.UpdateDeltaTime(new_delta_time);

        // Compute throttle control command to apply
        pid_throttle.Update(velocity_setpoint, velocity);

        // Get PID throttle control command
        double throttle_ctrl_cmd = pid_throttle.GetControlCommand();

        // Get PID throttle control errors and error gains
        pid_throttle_errors = pid_throttle.GetErrors();
        pid_throttle_error_gains = pid_throttle.GetErrorGains();

        // Decompose pid throttle error vector
        double prop_velocity_error = pid_throttle_errors[0];
        double int_velocity_error = pid_throttle_errors[1];
        double diff_velocity_error = pid_throttle_errors[2];

        // Decompose pid throttle error gain vector
        double prop_throttle_error_gain = pid_throttle_error_gains[0];
        double int_throttle_error_gain = pid_throttle_error_gains[1];
        double diff_throttle_error_gain = pid_throttle_error_gains[2];

        // Define throttle and brake control commands
        double throttle_output, brake_output;

        // Adapt the negative throttle to break
        if (throttle_ctrl_cmd > 0.0) {
          throttle_output = throttle_ctrl_cmd;
          brake_output = 0;
        } else {
          throttle_output = 0;
          brake_output = -throttle_ctrl_cmd;
        }

        // Save longitudinal control data
        file_throttle.seekg(std::ios::beg);
        for (int j=0; j < i - 1; ++j) {
          file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
        }
        file_throttle << i ;
        file_throttle << " " << velocity_setpoint;
        file_throttle << " " << velocity;
        file_throttle << " " << prop_velocity_error;
        file_throttle << " " << int_velocity_error;
        file_throttle << " " << diff_velocity_error;
        file_throttle << " " << prop_throttle_error_gain;
        file_throttle << " " << int_throttle_error_gain;
        file_throttle << " " << diff_throttle_error_gain;
        file_throttle << " " << throttle_ctrl_cmd;
        file_throttle << " " << throttle_output;
        file_throttle << " " << brake_output;
        file_throttle << " " << velocity_closest_wp;
        file_throttle << " " << velocity_lookahead_wp << endl;

        // Initialize json message
        json msgJson;

        // Send control signal
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

        //  min point threshold before doing the update
        // for high update rate use 19 for slow update rate use 4
        msgJson["update_point_thresh"] = 16;

        // Dump json message containing the updated data from pid_controller
        auto msg = msgJson.dump();

        // Increment simulation time step
        i = i + 1;

        // Close log files with the pid_controller data
        file_steer.close();
        file_throttle.close();

        // Send data back to simulatiorAPI
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

      }
    }
  );


  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      cout << "Connected!!!" << endl;
    }
  );


  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
      ws.close();
      cout << "Disconnected" << endl;
    }
  );

  int port = 4567;
  if ( h.listen("0.0.0.0", port) ) {
    cout << "Listening to port " << port << endl;
    h.run();
  } else {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }

}
