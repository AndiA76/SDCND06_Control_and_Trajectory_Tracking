/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: January 7, 2023
 *      Author: Andreas Albrecht
 **********************************************/

#ifndef STANLEY_CONTROLLER_H
#define STANLEY_CONTROLLER_H

#include <vector>

using namespace std;


class STANLEY {
public:
    /**
    * Create the STANLEY controller class
    **/

    /*
    * Errors
    */
    double heading_error_;
    double crosstrack_error_;
    double crosstrack_heading_error_;   // cross-track error projected to a heading error

    /*
    * Control parameters
    */
    double crosstrack_gain_;     // crosstrack_error gain (weighting factor w.r.t. heading_error)
    double velocity_threshold_;  // velocity threshold to avoid division by zero for low velocities

    /*
    * Output limits
    */
    double output_lim_min_;     // lower output limit
    double output_lim_max_;     // lower output limit

    /*
    * Control ouptut
    */
    double control_output_ = 0.0;

    /*
    * Constructor
    */
    STANLEY();

    /*
    * Destructor.
    */
    virtual ~STANLEY();

    /*
    * Initialize Stanley controller.
    */
    void Init(double crosstrack_gain, double output_lim_min, double output_lim_max, double velocity_threshold = 1.0e-6);

    /*
    * Update the Stanley control steer command given heading setpoint, actual heading, crosstrack error and velocity.
    */
    void Update(double heading_setpoint, double actual_heading, double crosstrack_error, double velocity);

    /*
    * Get the steer control command.
    */
    double GetControlCommand();

    /*
    * Get the Stanley control error gains.
    */
    vector<double> GetErrorGains();

    /*
    * Get the Stanley control errors.
    */
    vector<double> GetErrors();

};

#endif //STANLEY_CONTROLLER_H


