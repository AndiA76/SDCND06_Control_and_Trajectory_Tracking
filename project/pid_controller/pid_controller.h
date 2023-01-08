/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: January 7, 2023
 *      Author: Andreas Albrecht
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:
    /**
    * Create the PID class
    **/

    /*
    * Errors
    */
    double curr_error_;     // current error
    double prev_error_;     // previous error
    double prop_error_;     // proporional part
    double int_error_;      // integral part
    double diff_error_;     // differential part

    /*
    * Coefficients
    */
    double FF_;     // feed forward (lookahead) gain
    double Kp_;     // proportional gain
    double Ki_;     // integral gain
    double Kd_;     // differential gain

    /*
    * Output limits
    */
    double output_lim_min_;     // lower limit
    double output_lim_max_;     // upper limit

    /*
    * Control ouptut
    */
    double control_output_;

    /*
    * Delta time
    */
    double delta_t_;
    double delta_t_min_ = 1.0e-9;  // prevent division by zero

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double FF, double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min, double int_error_0 = 0.0);

    /*
    * Update the PID error variables given the actual error (e.g. cross-track or velocity error).
    */
    void UpdateError(double actual_error);

    /*
    * Get the PID control command.
    */
    double GetControllCommand();
  
    /*
    * Update the delta time.
    */
    void UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H


