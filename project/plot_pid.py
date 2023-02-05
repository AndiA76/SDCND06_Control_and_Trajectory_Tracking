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
        usecols = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21]
    )
    steer_df.columns = [
        'Iteration',
        'Steer Setpoint',
        'Actual Steer Value',
        'Proportional Steer Error',
        'Integral Steer Error',
        'Differential Steer Error',
        'Steer Control Command',
        'Yaw to Lookahead Waypoint',
        'Yaw to Closest Waypoint',
        'Actual Yaw',
        'Heading Error w.r.t. Lookahead Waypoint',
        'Heading Error w.r.t. Closest Waypoint',
        'Crosstrack Error w.r.t. Lookahead Waypoint',
        'Crosstrack Error w.r.t. Closest Waypoint',
        'Actual X-Position',
        'Actual Y-Position',
        'Lookahead X-Waypoint',
        'Lookahead Y-Waypoint',
        'Closest X-Waypoint',
        'Closest Y-Waypoint',
        'Distance to Lookahead Waypoint',
        'Distance to Closest Waypoint',
    ]
    print("Steer data:")
    print(steer_df.head())
    print(steer_df.columns)
    return steer_df


def read_throttle_data():
    """Read recorded longitudinal control data."""
    throttle_file = 'throttle_pid_data.txt'
    throttle_df = pd.read_csv(
        throttle_file, delim_whitespace = True, header = None, usecols = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    )
    throttle_df.columns = [
        'Iteration',
        'Velocity Setpoint',
        'Actual Velocity',
        'Velocity Error',
        'Integral Velocity Error',
        'Differential Velocity Error',
        'Throttle Control Command',
        'Throttle Output',
        'Brake Output',
        'Planned Velocity (Lookahead Point)',
        'Planned Velocity (Closest Point)',
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
            'Steer Setpoint',
            'Actual Steer Value',
            'Proportional Steer Error',
            'Integral Steer Error',
            'Differential Steer Error',
        ]
    )
    ax1.set_title('Steering control errors')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Steer error [--]')

    # Plot steering control ouptut
    steer_ctr_df2.plot(
        ax = ax2,
        x = 'Iteration',
        y = 'Steer Control Command'
    )
    ax2.set_title('Steer control command')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Steer control output [1]')

    # Plot crosstrack error
    steer_ctr_df2.plot(
        ax = ax3,
        x = 'Iteration',
        y = [
            'Crosstrack Error w.r.t. Lookahead Waypoint',
            'Crosstrack Error w.r.t. Closest Waypoint',
        ]
    )
    ax3.set_title('Crosstrack errors (steer control)')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Crosstrack error [m]')

    # Plot heading errors
    steer_ctr_df2.plot(
        ax = ax4,
        x = 'Iteration',
        y = [
            'Heading Error w.r.t. Lookahead Waypoint',
            'Heading Error w.r.t. Closest Waypoint',
        ]
    )
    ax4.set_title('Heading errors (steer control)')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Heading error [rad]')

    fig.suptitle('Lateral control (steer control)')


def plot_throttle_data(throttle_ctr_df, num_rows):
    """Plot recorded longitudinal control (throttle control) data."""

    # Define subplot layout
    fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1)

    # Get first n_rows from throttle dataframe
    throttle_ctr_df2 = throttle_ctr_df[:num_rows]

    print('throttle_df2.columns')
    print(throttle_ctr_df2.columns)

    # Plot velocity control errors
    throttle_ctr_df2.plot(
        ax = ax1,
        x = 'Iteration',
        y = [
            'Velocity Setpoint',
            'Actual Velocity',
            'Velocity Error',
            'Integral Velocity Error',
            'Differential Velocity Error'
        ]
    )
    ax1.set_title('Throttle control errors')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Throttle and brake output [1]')

    # Plot throttle control ouptut
    throttle_ctr_df2.plot(
        ax = ax2,
        x = 'Iteration',
        y = [
            'Throttle Control Command',
            'Throttle Output',
            'Brake Output',
        ]
    )
    ax2.set_title('Throttle & brake control command')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Throttle & brake output [1]')

    fig.suptitle('Longitudinal control (throttle control)')


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
            'Lookahead X-Waypoint',
            'Closest X-Waypoint',
        ]
    )
    ax1.set_title('X-coordinates of actual and planned trajectory')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('X-cooridnate in [m]')

    # Plot y-coordinates of actual and planned path
    steer_ctr_df2.plot(
        ax = ax2,
        x = 'Iteration',
        y = [
            'Actual Y-Position',
            'Lookahead Y-Waypoint',
            'Closest Y-Waypoint',
        ]
    )
    ax2.set_title('Y-coordinates of actual and planned trajectory')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Y-cooridnate in [m]')

    # Plot distances between actual position and planned path waypoints
    steer_ctr_df2.plot(
        ax = ax3,
        x = 'Iteration',
        y = [
            'Distance to Lookahead Waypoint',
            'Distance to Closest Waypoint',
        ]
    )
    ax3.set_title('Distances between actual position and planned path waypoints')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Distance in [m]')

    steer_ctr_df2.plot(
        ax = ax4,
        x = 'Iteration',
        y = [
            'Actual Yaw',
            'Yaw to Lookahead Waypoint',
            'Yaw to Closest Waypoint',
        ]
    )
    ax4.set_title('Heading deviation')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Heading direction [rad]')

    fig.suptitle('Path coordinates of actual and planned trajectories')


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

    # Plot planned trajectory w.r.t. to lookahead waypoint
    steer_ctr_df2.plot(
        ax = ax1,
        x = 'Lookahead X-Waypoint',
        y = [
            'Lookahead Y-Waypoint',
        ],
        color='blue'
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
