/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: January 7, 2023
 *      Author: Andreas Albrecht
 **********************************************/

#include "stanley_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

STANLEY::STANLEY() {}

STANLEY::~STANLEY() {}

void STANLEY::Init(double crosstrack_gain, double output_lim_min, double output_lim_max, double velocity_threshold) {
   /**
   * Initialize Stanley control paramters
   **/
   // Set crosstrack_error gain
   crosstrack_gain_ = crosstrack_gain;
   // Set velocity threshold
   velocity_threshold_ = velocity_threshold;
   // Set output saturation limits
   output_lim_min_ = output_lim_min;
   output_lim_max_ = output_lim_max;
}

void STANLEY::Update(double heading_setpoint, double actual_heading, double crosstrack_error, double velocity) {
   /**
   * Update Stanley steering control errors and control command given heading setpoint, actual heading, actual cross-track error and velocity.
   **/

   // Calculate current heading error
   heading_error_ = heading_setpoint - actual_heading;

   // Store current cross-track error
   crosstrack_error_ = crosstrack_error;

   // Transform crosstrack error into a heading error component
   crosstrack_heading_error_ = atan(crosstrack_gain_ * crosstrack_error_ / (velocity + velocity_threshold_));

   // Calculate Stanley control command
   control_output_ = heading_error_ + crosstrack_heading_error_;

   // Limit control output to its lower and upper saturation limits
   if (control_output_ < output_lim_min_) {
      control_output_ = output_lim_min_;
   } else if (control_output_ > output_lim_max_) {
      control_output_ = output_lim_max_;
   }
}

double STANLEY::GetControlCommand() {
   /**
   * Get the Stanley steering control command bound to the interval [output_lim_min_, output_lim_max_]
   */
   return control_output_;
}

vector<double> STANLEY::GetErrorGains() {
   /**
   * Get the STANLEY steering control error gains as a vector<double> = {heading_error_, crosstrack_heading_error_}
   */
   vector<double> output_errors_gains_ = {heading_error_, crosstrack_heading_error_};
   return output_errors_gains_;
}

vector<double> STANLEY::GetErrors() {
   /**
   * Get the STANLEY control errors as a vector<double> = {heading_error_, crosstrack_error_}
   */
   vector<double> output_errors_ = {heading_error_, crosstrack_error_};
   return output_errors_;
}
