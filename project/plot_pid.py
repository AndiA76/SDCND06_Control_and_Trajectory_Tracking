"""Script to plot longitudinal and lateral control errors and control commands."""
# **********************************************
#  Self-Driving Car Nano-degree - Udacity
#   Created on: January 7, 2023
#       Author: Andreas Albrecht
# **********************************************

import pandas as pd
import matplotlib.pyplot as plt


def read_steer_data():
    """Read recorded lateral control data."""
    steer_file = 'steer_pid_data.txt'
    steer_df = pd.read_csv(
        steer_file,
        delim_whitespace = True,
        header = None,
        usecols = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
    )
    steer_df.columns = [
        'Iteration',
        'Yaw Setpoint',
        'Actual Yaw',
        'Heading Error',
        'Integral Heading Error', 
        'Diffential Heading Error',
        'Steering Control Command',
        'Actual X-Position',
        'Actual Y-Position',
        'Lookahead X-Waypoint',
        'Lookahead Y-Waypoint',
        'Closest X-Waypoint',
        'Closest Y-Waypoint',
        'Distance to Lookahead Waypoint',
        'Distance to Closest Waypoint',
        'Yaw to Closest Waypoint',
    ]
    print("Steer data:")
    print(steer_df.head())
    return steer_df


def read_throttle_data():
    """Read recorded longitudinal control data."""
    throttle_file = 'throttle_pid_data.txt'
    throttle_df = pd.read_csv(
        throttle_file, delim_whitespace = True, header = None, usecols = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    )
    throttle_df.columns = [
        'Iteration',
        'Velocity Setpoint',
        'Actual Velocity',
        'Velocity Error',
        'Integral VelocityError',
        'Differential VelocityError',
        'Throttle Control Command',
        'Throttle Output',
        'Brake Output',
        'Planned Velocity (Closest Point)',
    ])
    print("Throttle data:")
    print(throttle_df.head())
    return throttle_df


def plot_steer_data(steer_df, n_rows):
    """Plot recorded lateral control (steer control) data."""
    steer_df2 = steer_df[:n_rows]
    steer_df2.plot(
        x = steer_df.columns.get_loc['Iteration'],
        y = [
            steer_df.columns.get_loc['Yaw Setpoint'],
            steer_df.columns.get_loc['Actual Yaw'],
            steer_df.columns.get_loc['Heading Error'],
            steer_df.columns.get_loc['Integral Heading Error'],
            steer_df.columns.get_loc['Differential Heading Error'],
            steer_df.columns.get_loc['Steering Control Command']
        ]
        kind = 'line',
        label = [
            'yaw setpoint',
            'actual yaw',
            'heading error',
            'integral heading error',
            'differential heading error',
            'steering control command'
        ]
    )
    plt.title('Lateral control (steer control)')
    plt.xlabel('Time [s]')
    plt.ylabel('Control error | Control command [1]')
    plt.show()

    
def plot_throttle_data(throttle_df, n_rows):
    """Plot recorded longitudinal control (throttle control) data."""
    throttle_df2 = throttle_df[:n_rows]
    throttle_df2.plot(
        x = throttle_df.columns.get_loc['Iteration'],
        y = [
            throttle_df.columns.get_loc['Velocity Setpoint'],
            throttle_df.columns.get_loc['Actual Velocity'],
            throttle_df.columns.get_loc['Velocity Error'],
            throttle_df.columns.get_loc['Integral Velocity Error'],
            throttle_df.columns.get_loc['Differential Velocity Error'],
            throttle_df.columns.get_loc['Throttle Control Command'],
            throttle_df.columns.get_loc['Throttle Output'],
            throttle_df.columns.get_loc['Brake Output'],
        ]
        kind = 'line',
        label = [
            'velocity setpoint',
            'actual velocity',
            'velocity error',
            'integral velocity error',
            'differential velocity error',
            'throttle control command',
            'throttle output',
            'brake command',
        ]
    )
    plt.title('Longitudinal control (throttle control)')
    plt.xlabel('Time [s]')
    plt.ylabel('Control error | Control command [1]')
    plt.show()


def plot_velocities(throttle_df, n_rows):
    """Plot recorded planned and actual velocity data."""
    throttle_df2 = throttle_df[:n_rows]
    throttle_df2.plot(
        x = throttle_df.columns.get_loc['Iteration'],
        y = [
            throttle_df.columns.get_loc['Velocity Setpoint'],
            throttle_df.columns.get_loc['Planned Velocity (Closest Point)'],
            throttle_df.columns.get_loc['Actual Velocity'],
        ],
        kind = 'line',
        label = ['planned velocity (lookahead waypoint)', 'planned velocity (closest waypoint)', 'actual velocity']
    )
    plt.title('Planned and actual veolocity data')
    plt.xlabel('Time [s]')
    plt.ylabel('Veolocity [m/s]')
    plt.show()

    
def main():
    """Read and plot recorded lateral and longitudinal control data."""
    steer_df = read_steer_data()
    throttle_df = read_throttle_data()
    n_rows = -1 #2000
    plot_steer_data(steer_df, n_rows)
    plot_throttle_data(throttle_df, n_rows)
    plot_velocities(throttle_df, n_rows)


if __name__ == '__main__':
    main()
