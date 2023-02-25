"""Implementation of PID lateral robot controller optimized using twiddle."""
# ----------------------------------------------------------------------
# Self-Driving Car Nano-degree - Udacity (https://www.udacity.com/)
# 
# Modified by Andreas Albrecht
# ----------------------------------------------------------------------

import random
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple


class Robot(object):
    """Robot class using a simplified bicycle model for the 2D kinematics."""
    def __init__(self, wheelbase: float = 20.0):
        """Create robot and initialize x/y-location & orientation to 0, 0, 0.

        Args:
            wheelbase (float): distance between front and rear axle center
                Must be greater zero. Defautls to 20.0.
        """
        # Initialize 2D position of the rear axle center point
        self.x_pos = 0.0
        self.y_pos = 0.0
        # Initialize robot orientation (yaw angle)
        self.yaw = 0.0
        # Set wheelbase (= distance between steered front and fixed rear axle)
        self.wheelbase = wheelbase
        # Initialize noise and drift parameters
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, x_pos: float, y_pos: float, yaw: float):
        """Set 2D position and orientation of the robot.

        Args:
            x_pos (float): x-coordinate of the robot's location
            y_pos (float): x-coordinate of the robot's location
            orientation (float): y_posaw angle / orientation of the robot
        """
        # Set rear axle position
        self.x_pos = x_pos
        self.y_pos = y_pos
        # Set orientation wrapping it to [-pi, +pi]
        # self.yaw = (yaw + np.pi) % (2.0 * np.pi) - np.pi
        self.yaw = yaw % (2.0 * np.pi)

    def set_noise(self, steering_noise: float, distance_noise: float):
        """Set noise parameters on steering angle and distance measurement.

        Args:
            steering_noise (float): noise on steering angle measurement
            distance_noise (float): noise on distance measurement
        """
        # Set measurement noise parameters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift: float):
        """Set some systematical steering drift parameter.

        Args:
            drift (float): error overlay to create systematic steering drift
        """
        self.steering_drift = drift

    def move(
        self,
        steering_angle: float,
        distance_driven: float,
        tolerance: float = 0.001,
        max_steering_angle: float = np.pi / 4.0
    ):
        """Move robot along its trajectory within an incremental time step.

        Args:
            steering_angle (float): front wheel steering angle in [rad]
            distance_driven (float): total distance driven (must be > 0)
            tolerance (float): tolerance to approximate a straight-line motion
            max_steering_angle (float): maximum steering angle in [rad]
        """
        # Limit steering angle
        steering_angle = max(
            min(
                steering_angle,
                max_steering_angle
            ),
            -max_steering_angle
        )
        # Only consider driving in positive direction
        distance_driven = max(0.0, distance_driven)

        # Apply measurement noise
        noisy_steering_angle = random.gauss(
            steering_angle, self.steering_noise
        )
        noisy_distance_driven = random.gauss(
            distance_driven, self.distance_noise
        )

        # Apply steering drift
        noisy_steering_angle += self.steering_drift

        # Calculate turn angle about the instantaneous center of rotiation
        turn_angle = noisy_distance_driven * \
            np.tan(noisy_steering_angle) / self.wheelbase

        if abs(turn_angle) < tolerance:
            # Approximate straight line motion
            self.x_pos += noisy_distance_driven * np.cos(self.yaw)
            self.y_pos += noisy_distance_driven * np.sin(self.yaw)
            # Upate yaw by adding turn angle in radians wrapping it to [-pi, +pi]
            # self.yaw = (self.yaw + turn_angle + np.pi) % (2.0 * np.pi) - np.pi
            self.yaw = (self.yaw + turn_angle) % (2.0 * np.pi)
        else:
            # Calculate turn ratius about the instantaneous center of rotation
            turn_radius = noisy_distance_driven / turn_angle
            # Previous yaw angle
            prev_yaw = self.yaw
            # Upate yaw by adding turn angle in radians wrapping it to [-pi, +pi]
            #self.yaw = (self.yaw + turn_angle + np.pi) % (2.0 * np.pi) - np.pi
            self.yaw = (self.yaw + turn_angle) % (2.0 * np.pi)
            # Cacluate robot motion using 2D bicycle model kinematics
            self.x_pos += turn_radius * (np.sin(self.yaw) - np.sin(prev_yaw))
            self.y_pos += turn_radius * (np.cos(prev_yaw) - np.cos(self.yaw))

    def __repr__(self):
        """Return the robot's current location and orientation as a string."""
        return f"[x_pos = {self.x_pos:.5f}, y_pos = {self.y_pos:.5f}, yaw = {self.yaw:.5f}]"


def make_robot() -> Robot:
    """Create a robot setting all states back to initial values.

    Returns:
        robot object (Robot)
    """
    # Create a robot object
    robot_obj = Robot()
    # Set its initial robot position
    robot_obj.set(0, 1, 0)
    # Set steering drift
    robot_obj.set_steering_drift(10 / 180 * np.pi)
    return robot_obj


def run(
    robot_model: Robot,
    pid_params: Tuple[float],
    n_steps: int = 100,
    speed: float = 1.0,
    time_step: float = 1.0
) -> Tuple[List[float], List[float], float]:
    """Run a simulation returning the cross-track error.

    We assume a straight motion of the robot along the x-axis.

    Args:
        robot_model (Robot): a robot model
        pid_params (Tuple[float]): control parameters to be optimized
        n_steps (int): number of simulation time steps
        speed (float): robot speed (assumed to be constant)

    Returns:
        2D trajectory points and mean square error
    """
    # Initialize lists to store the trajectory points
    x_traj = []
    y_traj = []
    # Initialize mean-squared-error (mse)
    mse = 0
    # Initialize previous cross-track error (cte)
    prev_cte = robot_model.y_pos
    # Initialize cross-track error integral (int_cte)
    int_cte = 0
    for i in range(2 * n_steps):
        # Get current cross-track error (corresponds to the y-position in this case)
        cte = robot_model.y_pos
        # Differentiate cross-track error
        diff_cte = cte - prev_cte
        # Integrate cross-track error
        int_cte += cte
        # Remember current cross-track error for the next step
        prev_cte = cte
        # Calculate steering command using a linear PID controller
        steer_command = -(
            pid_params[0] * cte + pid_params[1] * diff_cte + pid_params[2] * int_cte
        )
        # Calculate the driven distance
        driven_distance = speed * time_step
        # Move the robot one step further
        robot_model.move(steer_command, driven_distance)
        # Store the trajectory points
        x_traj.append(robot_model.x_pos)
        y_traj.append(robot_model.y_pos)
        # Accumulate squared error after first n_steps (excluding transients)
        if i >= n_steps:
            mse += cte ** 2
    # Calculate the mean squared error after second n_steps
    mse /= n_steps
    # Return trajectory points and mean squared error
    return x_traj, y_traj, mse


# Make this tolerance bigger if you are timing out!
def twiddle(tol: float = 0.2) -> Tuple[float]:
    """Twiddle algorithm to optimize PID control parameters.
    
    Args:
        tol (float): tolerance for changing pid parameters
    """

    # Remark: Call `make_robot()` before every call of `run()`!

    # Result lists
    results = {
        "pid_parameter": [],
        "x_trajectory": [],
        "y_trajectory": [],
        "mse": [],
    }

    # Initial parameter vector p
    p = [0., 0., 0.]
    # Initial vector of potential parameter changes
    dp = [1., 1., 1.]
    # Create and run a robot to obtain an initial error
    new_robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(new_robot, p)
    # Store initial results
    results['pid_parameter'].append(p)
    results['x_trajectory'].append(x_trajectory)
    results['y_trajectory'].append(y_trajectory)
    results['mse'].append(best_err)
    # Twiddle optimization loop
    iter = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(iter, best_err))
        for i in range(len(p)):
            # Increase the parameter p[i] by dp[i]
            p[i] += dp[i]

            # Create and run new robot to obtain a new error
            new_robot = make_robot()
            x_trajectory, y_trajectory, err = run(new_robot, p)
            # Store intermediate results
            results['pid_parameter'].append(p)
            results['x_trajectory'].append(x_trajectory)
            results['y_trajectory'].append(y_trajectory)
            results['mse'].append(err)

            # Check if the error could be reduced witerh the new parameter
            if err < best_err:
                # Update best error
                best_err = err

                # If last change was successful increase p[i] by another 10%
                dp[i] *= 1.1
            else:
                # If the last change was not successful change p[i] by dp[i] in the oposite direction
                p[i] -= 2 * dp[i]  # now we need to subtract dp[i] twice as it was added once before

                # Create and run new robot to obtain a new error
                new_robot = make_robot()
                x_trajectory, y_trajectory, err = run(new_robot, p)
                # Store intermediate results
                results['pid_parameter'].append(p)
                results['x_trajectory'].append(x_trajectory)
                results['y_trajectory'].append(y_trajectory)
                results['mse'].append(err)

                # Check if the error could be reduced with the new parameter
                if err < best_err:
                    # Update best error
                    best_err = err

                    # If last change was successful increase p[i] by another 10%
                    dp[i] *= 1.1
                else:
                    # If above changes fail set p[i] back to its original value
                    p[i] += dp[i]
                    # and decrease parameter change by 10%
                    dp[i] *= 0.9
        # Update number of iterations
        iter += 1
        print("p = {}, dp = {}".format(p, dp))
    return p, best_err, results


def main():
    """Optimize PID robot motion control parameters using twiddle."""
    pid_params_opt, mse_min, res = twiddle()
    print(f"Final twiddle mean squared error = {mse_min}")
    print(f"Optimized PID parameter [Kp, Kd, Ki] = [{pid_params_opt}]")
    twiddle_robot = make_robot()
    x_trajectory, y_trajectory, mse = run(twiddle_robot, pid_params_opt)
    n = len(x_trajectory)

    # Plot optimization results
    fig0, ax0 = plt.subplots(1, 1, figsize=(8, 4))
    ax0.plot(x_trajectory, np.zeros(n), 'k', linestyle='dashdot', linewidth=1, label='reference')
    ax0.plot(x_trajectory, y_trajectory, 'g', linestyle='solid', linewidth=1, label='twiddle PID controller')
    ax0.set_xlabel('Number of steps')
    ax0.set_ylabel('Cross-track error')
    fig0.suptitle('Robot trajectory twiddle-optimized PID control parameters')

    # Plot optimal trajectory
    fig1, (ax11, ax12) = plt.subplots(2, 1, figsize=(8, 8))
    ax11.plot(x_trajectory, np.zeros(n), 'k', linestyle='dashdot')
    for x_traj, y_traj in zip(res['x_trajectory'], res['y_trajectory']):
        # Generate random color
        random_color = list(np.random.choice(range(256), size=3))
        # Plot trajectory
        ax11.plot(x_traj, y_traj, random_color, linestyle='dotted', linewidth=1)
    ax11.set_ylabel('Cross-track error')
    ax11.set_title('Trajectories of all PID optimization runs')
    ax12.plot(x_trajectory, np.zeros(n), 'k', linestyle='dashdot', linewidth=1, label='reference (set value)')
    ax12.plot(x_trajectory, y_trajectory, 'g', linestyle='solid', linewidth=1, label='twiddle-optimized PID')
    ax12.set_xlabel('Number of steps')
    ax12.set_ylabel('Cross-track error')
    ax12.set_title('Trajectory with optimized PID parameters')
    fig1.suptitle('Robot trajectory before and after twiddle optimization')
    plt.show()


if __name__ == '__main__':
    main()
