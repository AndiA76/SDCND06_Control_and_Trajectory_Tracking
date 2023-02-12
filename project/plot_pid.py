"""Script to plot longitudinal and lateral control errors and control commands."""
# **********************************************
#  Self-Driving Car Nano-degree - Udacity
#   Created on: January 7, 2023
#       Author: Andreas Albrecht
# **********************************************

import os
import pandas as pd
import matplotlib.pyplot as plt

#FILEPATH = 'experiments/test_cte_07/'
FILEPATH = 'experiments/test_yaw_02/'


def read_steer_data():
    """Read recorded lateral control data."""
    steer_filepath = os.path.join(FILEPATH, 'steer_pid_data.txt')
    cols = pd.read_csv(steer_filepath, delim_whitespace = True, nrows = 1).columns
    steer_df = pd.read_csv(
        steer_filepath, delim_whitespace = True, header = None, usecols = list(range(0, len(cols)))
    )
    steer_df.columns = [
        'Iteration',
        'Steer Setpoint Value',
        'Actual Steer Value',
        'Proportional Steer Error',
        'Integral Steer Error',
        'Differential Steer Error',
        'Proportional Steer Error Gain',
        'Integral Steer Error Gain',
        'Differential Steer Error Gain',
        'Steer Control Command',
        'Yaw to Closest Waypoint',
        'Yaw to Lookahead Waypoint',
        'Actual Yaw',
        'Heading Error w.r.t. Closest Waypoint',
        'Heading Error w.r.t. Lookahead Waypoint',
        'Crosstrack Error w.r.t. Closest Waypoint',
        'Crosstrack Error w.r.t. Lookahead Waypoint',
        'Actual X-Position',
        'Actual Y-Position',
        'Closest X-Waypoint',
        'Closest Y-Waypoint',
        'Lookahead X-Waypoint',
        'Lookahead Y-Waypoint',
        'Distance to Closest Waypoint',
        'Distance to Lookahead Waypoint',
    ]
    print("Steer data:")
    print(steer_df.head())
    print(steer_df.columns)
    return steer_df


def read_throttle_data():
    """Read recorded longitudinal control data."""
    throttle_filepath = os.path.join(FILEPATH, 'throttle_pid_data.txt')
    cols = pd.read_csv(throttle_filepath, delim_whitespace = True, nrows = 1).columns
    throttle_df = pd.read_csv(
        throttle_filepath, delim_whitespace = True, header = None, usecols = list(range(0, len(cols)))
    )
    throttle_df.columns = [
        'Iteration',
        'Velocity Setpoint',
        'Actual Velocity',
        'Proportional Velocity Error',
        'Integral Velocity Error',
        'Differential Velocity Error',
        'Proportional Throttle Error Gain',
        'Integral Throttle Error Gain',
        'Differential Throttle Error Gain',
        'Throttle Control Command',
        'Throttle Output',
        'Brake Output',        
        'Planned Velocity (Closest Point)',
        'Planned Velocity (Lookahead Point)',
    ]
    print("Throttle data:")
    print(throttle_df.head())
    print(throttle_df.columns)
    return throttle_df


def plot_steer_data(steer_ctr_df, num_rows):
    """Plot recorded lateral control (steer control) data."""

    # Define subplot layout
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(nrows=4, ncols=1)

    # Get first n_rows from steering dataframe
    steer_ctr_df2 = steer_ctr_df[:num_rows]

    # Plot steering control errors
    steer_ctr_df2.plot(
        ax = ax1,
        x = 'Iteration',
        y = [
            'Steer Setpoint Value',
            'Actual Steer Value',
            'Proportional Steer Error',
            'Integral Steer Error',
            'Differential Steer Error',
        ]
    )
    ax1.set_title('Steering control errors')
    ax1.set_xlabel('')
    ax1.set_ylabel('Steer error [-]')

    # Plot steering control ouptut
    steer_ctr_df2.plot(
        ax = ax2,
        x = 'Iteration',
        y = [
            'Proportional Steer Error Gain',
            'Integral Steer Error Gain',
            'Differential Steer Error Gain',
            'Steer Control Command',
        ]
    )
    ax2.set_title('Steer control command')
    ax2.set_xlabel('')
    ax2.set_ylabel('Steer control output [1]')

    # Plot crosstrack error
    steer_ctr_df2.plot(
        ax = ax3,
        x = 'Iteration',
        y = [
            'Crosstrack Error w.r.t. Closest Waypoint',
            'Crosstrack Error w.r.t. Lookahead Waypoint',
        ]
    )
    ax3.set_title('Crosstrack errors (steer control)')
    ax3.set_xlabel('')
    ax3.set_ylabel('Crosstrack error [m]')

    # Plot heading errors
    steer_ctr_df2.plot(
        ax = ax4,
        x = 'Iteration',
        y = [
            'Heading Error w.r.t. Closest Waypoint',
            'Heading Error w.r.t. Lookahead Waypoint',
        ]
    )
    ax4.set_title('Heading errors (steer control)')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Heading error [rad]')

    # Set higher-level title of the figure
    fig.suptitle('Lateral control (steer control)')
    # Avoid overlapping of titles and axis labels in subplots
    plt.subplots_adjust(hspace=0.35)


def plot_throttle_data(throttle_ctr_df, num_rows):
    """Plot recorded longitudinal control (throttle control) data."""

    # Define subplot layout
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(nrows=4, ncols=1)

    # Get first n_rows from throttle dataframe
    throttle_ctr_df2 = throttle_ctr_df[:num_rows]

    print('throttle_df2.columns')
    print(throttle_ctr_df2.columns)

    # Plot throttle control errors
    throttle_ctr_df2.plot(
        ax = ax1,
        x = 'Iteration',
        y = [
            'Velocity Setpoint',
            'Actual Velocity',
            'Proportional Velocity Error',
            'Integral Velocity Error',
            'Differential Velocity Error',
        ]
    )
    ax1.set_title('Throttle control errors')
    ax1.set_xlabel('')
    ax1.set_ylabel('Velocity control errors [m/s]')

    # Plot throttle control ouptut
    throttle_ctr_df2.plot(
        ax = ax2,
        x = 'Iteration',
        y = [
            'Proportional Throttle Error Gain',
            'Integral Throttle Error Gain',
            'Differential Throttle Error Gain',
            'Throttle Control Command',
        ]
    )
    ax2.set_title('Throttle control command')
    ax2.set_xlabel('')
    ax2.set_ylabel('Throttle control output [1]')

    # Plot throttle & brake control ouptut
    throttle_ctr_df2.plot(
        ax = ax3,
        x = 'Iteration',
        y = [
            'Throttle Control Command',
            'Throttle Output',
            'Brake Output',
        ]
    )
    ax3.set_title('Throttle & brake control command')
    ax3.set_xlabel('')
    ax3.set_ylabel('Throttle & brake output [1]')

    # Plot velocity values
    throttle_ctr_df2.plot(
        ax = ax4,
        x = 'Iteration',
        y = [
            'Actual Velocity',
            'Planned Velocity (Closest Point)',
            'Planned Velocity (Lookahead Point)',
        ]
    )
    ax4.set_title('Velocity values (throttle control)')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Velocity [m/s]')

    # Set higher-level title of the figure
    fig.suptitle('Longitudinal control (throttle control)')
    # Avoid overlapping of titles and axis labels in subplots
    plt.subplots_adjust(hspace=0.35)


def plot_path_coordinates(steer_ctr_df, num_rows):
    """Plot recorded path coordinates of actual and planned trajectories."""

    # Define subplot layout
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(nrows=4, ncols=1)

    # Get first n_rows from steering dataframe
    steer_ctr_df2 = steer_ctr_df[:num_rows]

    # Plot x-coordinates of actual and planned path
    steer_ctr_df2.plot(
        ax = ax1,
        x = 'Iteration',
        y = [
            'Actual X-Position',
            'Closest X-Waypoint',
            'Lookahead X-Waypoint',
        ]
    )
    ax1.set_title('X-coordinates of actual and planned trajectory')
    ax1.set_xlabel('')
    ax1.set_ylabel('X-cooridnate in [m]')

    # Plot y-coordinates of actual and planned path
    steer_ctr_df2.plot(
        ax = ax2,
        x = 'Iteration',
        y = [
            'Actual Y-Position',
            'Closest Y-Waypoint',
            'Lookahead Y-Waypoint',
        ]
    )
    ax2.set_title('Y-coordinates of actual and planned trajectory')
    ax2.set_xlabel('')
    ax2.set_ylabel('Y-cooridnate in [m]')

    # Plot distances between actual position and planned path waypoints
    steer_ctr_df2.plot(
        ax = ax3,
        x = 'Iteration',
        y = [
            'Distance to Closest Waypoint',
            'Distance to Lookahead Waypoint',
        ]
    )
    ax3.set_title('Distances between actual position and planned path waypoints')
    ax3.set_xlabel('')
    ax3.set_ylabel('Distance in [m]')

    steer_ctr_df2.plot(
        ax = ax4,
        x = 'Iteration',
        y = [
            'Actual Yaw',
            'Yaw to Closest Waypoint',
            'Yaw to Lookahead Waypoint',
        ]
    )
    ax4.set_title('Heading deviation')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Heading direction [rad]')

    # Set higher-level title of the figure
    fig.suptitle('Path coordinates of actual and planned trajectories')
    # Avoid overlapping of titles and axis labels in subplots
    plt.subplots_adjust(hspace=0.35)


def plot_trajectories(steer_ctr_df, num_rows):
    """Plot recorded actual and planned trajectories."""

    # Define subplot layout
    fig, ax1 = plt.subplots(nrows=1, ncols=1)

    # Get first n_rows from steering dataframe
    steer_ctr_df2 = steer_ctr_df[:num_rows]

    # Plot actual trajectory
    steer_ctr_df2.plot(
        ax = ax1,
        x = 'Actual X-Position',
        y = [
            'Actual Y-Position',
        ],
        color='black'
    )

    # Plot planned trajectory w.r.t. to closest waypoint
    steer_ctr_df2.plot(
        ax = ax1,
        x = 'Closest X-Waypoint',
        y = [
            'Closest Y-Waypoint',
        ],
        color='green'
    )
    ax1.set_xlabel('X-coordinate [m]')
    ax1.set_ylabel('Y-cooridnate [m]')

    # Plot planned trajectory w.r.t. to lookahead waypoint
    steer_ctr_df2.plot(
        ax = ax1,
        x = 'Lookahead X-Waypoint',
        y = [
            'Lookahead Y-Waypoint',
        ],
        color='blue'
    )

    # Set higher-level title of the figure
    fig.suptitle('Actual and planned trajectories')


def main():
    """Read and plot recorded lateral and longitudinal control data."""
    steer_df = read_steer_data()
    print(steer_df.columns)
    throttle_df = read_throttle_data()
    print(throttle_df.columns)
    n_rows = -1 #2000
    plot_steer_data(steer_df, n_rows)
    plot_throttle_data(throttle_df, n_rows)
    plot_path_coordinates(steer_df, n_rows)
    plot_trajectories(steer_df, n_rows)
    plt.show()


if __name__ == '__main__':
    main()
