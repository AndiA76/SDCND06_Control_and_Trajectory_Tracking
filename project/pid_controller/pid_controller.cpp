/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: January 7, 2023
 *      Author: Andreas Albrecht
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;


PID::PID() {}

PID::~PID() {}

void PID::Init(
   double Kp,
   double Ki,
   double Kd,
   double output_lim_min,
   double output_lim_max,
   double int_win_min,
   double int_win_max,
   double delta_t_min,
   double initial_error
) {
   /**
   * Initialize PID coefficients, initial state of the integrator and output limits.
   **/
   // Set proportional gain
   Kp_ = Kp;
   // Set integral gain
   Ki_ = Ki;
   // Set derivative gain
   Kd_ = Kd;
   // Set output saturation limits
   output_lim_min_ = output_lim_min;
   output_lim_max_ = output_lim_max;
   // Set integration window limits
   int_win_min_ = int_win_min;
   int_win_max_ = int_win_max;
   // Set initial state of the integrator
   int_error_ = initial_error;
   // Set minmal delta time to prevent division by zero (limits differentiator magnitude)
   delta_t_min_ = delta_t_min;
   // Initialize current and previous error
   curr_error_ = initial_error;
   prev_error_ = curr_error_;
}

void PID::Update(double actual_setpoint, double actual_measurement) {
   /**
   * Update PID control errors and control command given the actual setpoint and the actual measurement.
   **/

   // Store the previous control error and calculate the actual control error
   prev_error_ = curr_error_;
   curr_error_ = actual_setpoint - actual_measurement;

   // Calculate differential error (prevent division by zero throught delta-time limitation)
   if (delta_t_ < delta_t_min_){
      diff_error_ = (curr_error_ - prev_error_) / delta_t_min_;
   } else {
      diff_error_ = (curr_error_ - prev_error_) / delta_t_;
   }

   // Predict integratal error using trapezoidal rule (before saturation)
   double pred_int_error = int_error_ + delta_t_/2 * (prev_error_ + curr_error_);

   // Predict PID control output (before saturation)
   double pred_control_output = Kp_ * curr_error_ + Ki_ * pred_int_error + Kd_ * diff_error_;

   // Anti-windup for integral part: Stop integration if control output saturates
   if (pred_control_output <= output_lim_min_) {
      // Clamp PID control output to its lower limit and stop integration (don't update int_error_)
      control_output_ = output_lim_min_;
   } else if (pred_control_output >= output_lim_max_) {
      // Clamp PID control output to its upper limit and stop integration (don't update int_error_)
      control_output_ = output_lim_max_;
   } else {
      // Update integral error and continue integration only within the allowed integration window
      if ((int_win_min_ < Ki_ * pred_int_error) && (Ki_ * pred_int_error < int_win_max_)) {
         int_error_ = pred_int_error;
      }
      // Update PID control output
      control_output_ = pred_control_output;
   }
}

double PID::GetControlCommand() {
   /**
   * Get the PID control command bound to the interval [output_lim_min_, output_lim_max_]
   */
   return control_output_;
}

vector<double> PID::GetErrorGains() {
   /**
   * Get the PID control error gains as a vector<double> = {Kp_ * curr_err_, Ki_ * int_error_, Kd_ * diff_error_}
   * 
   * Remark: Except for the I-component the PID errors are not clipped if saturation occurs!
   */
   vector<double> output_errors_gains_ = {Kp_ * curr_error_, Ki_ * int_error_, Kd_ * diff_error_};
   return output_errors_gains_;
}

vector<double> PID::GetErrors() {
   /**
   * Get the PID errors as a vector<double> = {curr_err_, int_error_, diff_error_}
   */
   vector<double> output_errors_ = {curr_error_, int_error_, diff_error_};
   return output_errors_;
}

void PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * Update the delta time with a new value
   */
   delta_t_ = new_delta_time;
}
