/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: January 7, 2023
 *      Author: Andreas Albrecht
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <vector>

using namespace std;


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
    double int_error_;      // integral error
    double diff_error_;     // differential error

    /*
    * Coefficients
    */
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
    double delta_t_min_ = 1.0e-6;  // prevent division by zero

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
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min, double int_error_0 = 0.0);

    /*
    * Update PID control errors and control command given the actual setpoint and the actual measurement.
    */
    void Update(double actual_setpoint, double actual_measurement);

    /*
    * Get the PID control command.
    */
    double GetControlCommand();

    /*
    * Get the PID control error gains.
    */
    vector<double> PID::GetErrorGains();

    /*
    * Get the PID control errors.
    */
    vector<double> GetErrors();
  
    /*
    * Update the delta time.
    */
    void UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H
