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

void PID::Init(double FF, double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min, double int_error_0 = 0.0) {
   /**
   * Initialize PID coefficients, initial state of the integrator and output limits.
   **/
   // Set feed forward (lookahead) gain
   FF_ = FF;
   // Set proportional gain
   Kp_ = Kp;
   // Set integral gain
   Ki_ = Ki;
   // Set derivative gain
   Kd_ = Kd;
   // Initialize current error
   curr_error_ = 0.0;
   // Set initial state of the integrator
   int_error_ = int_error_0;
   // Set output saturation limits
   output_lim_min_ = output_lim_min;
   output_lim_max_ = output_lim_max;
}

void PID::UpdateError(double actual_error) {
   /**
   * Update PID errors based on the actual error and calculate PID control command.
   **/
   // Update the actual and the previous error
   prev_error_ = curr_error_;
   curr_error_ = actual_error;

   // Calculate proportional error
   prop_error_ = curr_error_;

   // Calculate differential error (prevent division by zero)
   if (delta_t_ < delta_t_min_){
      diff_error_ = (curr_error_ - prev_error_) / delta_t_min_;
   } else {
      diff_error_ = (curr_error_ - prev_error_) / delta_t_;
   }

   // Predict integratal error using trapezoidal rule
   double pred_int_error = int_error_ + delta_t_/2 * (prev_error_ + curr_error_);

   // Predict PID control output
   double pred_control_output = FF_ + Kp_ * prop_error_ + Ki_ * pred_int_error + Kd_ * diff_error_;

   // Anti-windup for integral part
   if (pred_control_output < output_lim_min_) {
      // Set PID control output to its lower limit and stop integration (don't update int_error_)
      control_output_ = output_lim_min_;
   } else if (pred_control_output > output_lim_max_) {
      // Set PID control output to its upper limit and stop integration (don't update int_error_)
      control_output_ = output_lim_max_;
   } else {
      // Update integral error and continue integration
      int_error_ = pred_int_error;
      // Update PID control output
      control_output_ = pred_control_output;
   }
}

double PID::GetControllCommand() {
   /**
   * Get the PID control command bound to the interval [output_lim_min_, output_lim_max_]
   */
   // Return PID control command
   return control_output_;
}

void PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * Update the delta time with a new value
   */
   // Update time difference
   delta_t_ = new_delta_time;
}