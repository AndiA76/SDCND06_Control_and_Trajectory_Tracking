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

void STANLEY::Init(double crosstrack_gain, double velocity_threshold, double output_lim_max, double output_lim_min) {
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

double STANLEY::GetSteerCommand(double heading_error, double crosstrack_error, double velocity) {
   /**
   * Get Stanley control steering command based on mixed heading error and crosstrack error.
   **/

   // Calculate crosstrack steering command component
   double crosstrack_steer = atan(crosstrack_gain_ * crosstrack_error / (velocity + velocity_threshold_));

   // Remark: Heading steering command component is equal to heading error

   // Calculate Stanley control command
   double steer_control_output = heading_error + crosstrack_steer;

   // Limit control output to its saturation limits
   if (steer_control_output < output_lim_min_) {
      // Set control output to its lower limit
      steer_control_output = output_lim_min_;
   } else if (steer_control_output > output_lim_max_) {
      // Set control output to its upper limit
      steer_control_output = output_lim_max_;
   }

   // Return steer control comman
   return steer_control_output;
}