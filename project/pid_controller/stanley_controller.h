/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: January 7, 2023
 *      Author: Andreas Albrecht
 **********************************************/

#ifndef STANLEY_CONTROLLER_H
#define STANLEY_CONTROLLER_H

class STANLEY {
public:
    /**
    * Create the STANLEY controller class
    **/

    /*
    * Control parameters
    */
    double crosstrack_gain_;     // crosstrack_error gain (weighting factor w.r.t. heading_error)
    double velocity_threshold_;  // velocity threshold to avoid division by zero for low velocities

    /*
    * Output limits
    */
    double output_lim_min_;
    double output_lim_max_;

    /*
    * Constructor
    */
    STANLEY();

    /*
    * Destructor.
    */
    virtual ~STANLEY();

    /*
    * Initialize Stanley lateral control.
    */
    void Init(double crosstrack_gain, double velocity_threshold, double output_lim_max, double output_lim_min);

    /*
    * Update the Stanley control steer command given the actual heading_error, crosstrack_error and velocity.
    */
    double GetSteerCommand(double heading_error, double crosstrack_error, double velocity);
};

#endif //STANLEY_CONTROLLER_H


