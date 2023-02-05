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
        usecols = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]
    )
    steer_df.columns = [
        'Iteration',
        'Yaw Setpoint',
        'Actual Yaw',
        'Heading Error',
        'Integral Heading Error',
        'Differential Heading Error',
        'Steering Control Command',
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
        'Yaw to Lookahead Waypoint',
        'Yaw to Closest Waypoint',
    ]
    print("Steer data:")
    print(steer_df.head())
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
        'Planned Velocity (Lookahead Point)'
        'Planned Velocity (Closest Point)',
    ]
    print("Throttle data:")
    print(throttle_df.head())
    return throttle_df


def plot_steer_data(steer_df, n_rows):
    """Plot recorded lateral control (steer control) data."""

    # Define subplot layout
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1)

    # Get first n_rows from steering dataframe
    steer_df2 = steer_df[:n_rows]

    # Plot steering control errors
    steer_df2.plot(
        ax = ax1,
        x = steer_df.loc[:, 'Iteration'],
        y = [
            steer_df.loc[:, 'Yaw Setpoint'],
            steer_df.loc[:, 'Actual Yaw'],
            steer_df.loc[:, 'Heading Error'],
            steer_df.loc[:, 'Integral Heading Error'],
            steer_df.loc[:, 'Differential Heading Error'],
        ],
        kind = 'line',
        label = [
            'yaw setpoint',
            'actual yaw',
            'heading error',
            'integral heading error',
            'differential heading error',
        ]
    )
    ax1.set_title('Steering control errors')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Heading error [rad]')

    # Plot steering control ouptut
    steer_df2.plot(
        ax = ax2,
        x = steer_df.loc[:, 'Iteration'],
        y = [
            steer_df.loc[:, 'Steering Control Command'],
        ],
        kind = 'line',
        label = [
            'steering control command',
        ]
    )
    ax2.set_title('Steering control command')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Steer control output [1]')

    # Plot crosstrack error
    steer_df2.plot(
        ax = ax3,
        x = steer_df.loc[:, 'Iteration'],
        y = [
            steer_df.loc[:, 'Crosstrack Error w.r.t. Lookahead Waypoint'],
            steer_df.loc[:, 'Crosstrack Error w.r.t. Closest Waypoint'],
        ],
        kind = 'line',
        label = [
            'crosstrack error w.r.t. lookahead waypoint',
            'crosstrack error w.r.t. closest waypoint',
        ]
    )
    ax3.set_title('Crosstrack error (steer control)')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Crosstrack error [m]')

    fig.suptitle('Lateral control (steer control)')
    plt.show()

    
def plot_throttle_data(throttle_df, n_rows):
    """Plot recorded longitudinal control (throttle control) data."""

    # Define subplot layout
    fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1)

    # Get first n_rows from throttle dataframe
    throttle_df2 = throttle_df[:n_rows]

    # Plot velocity control errors
    throttle_df2.plot(
        ax = ax1,
        x = throttle_df.loc[:, 'Iteration'],
        y = [
            throttle_df.loc[:, 'Velocity Setpoint'],
            throttle_df.loc[:, 'Actual Velocity'],
            throttle_df.loc[:, 'Velocity Error'],
            throttle_df.loc[:, 'Integral Velocity Error'],
            throttle_df.loc[:, 'Differential Velocity Error'],
        ],
        kind = 'line',
        label = [
            'velocity setpoint',
            'actual velocity',
            'velocity error',
            'integral velocity error',
            'differential velocity error',
        ]
    )
    ax1.set_title('Throttle control errors')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Throttle and brake output [1]')

    # Plot throttle control ouptut
    throttle_df2.plot(
        ax = ax2,
        x = throttle_df.loc[:, 'Iteration'],
        y = [
            throttle_df.loc[:, 'Throttle Control Command'],
            throttle_df.loc[:, 'Throttle Output'],
            throttle_df.loc[:, 'Brake Output'],
        ],
        kind = 'line',
        label = [
            'throttle control command',
            'throttle output',
            'brake command',
        ]
    )
    ax2.set_title('Throttle & brake control command')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Throttle & brake output [1]')

    fig.suptitle('Longitudinal control (throttle control)')
    plt.show()


def plot_path_coordinates(steer_df, n_rows):
    """Plot recorded path coordinates of actual and planned trajectories."""

    # Define subplot layout
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1)

    # Get first n_rows from steering dataframe
    steer_df2 = steer_df[:n_rows]

    # Plot x-coordinates of actual and planned path
    steer_df2.plot(
        ax = ax1,
        x = steer_df.loc[:, 'Iteration'],
        y = [
            steer_df.loc[:, 'Actual X-Position'],
            steer_df.loc[:, 'Lookahead X-Waypoint'],
            steer_df.loc[:, 'Closest X-Waypoint'],
        ],
        kind = 'line',
        label = [
            'actual x-position',
            'lookahead x-waypoint',
            'closest x-waypoint',
        ]
    )
    ax1.set_title('X-coordinates of actual and planned trajectory')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('X-cooridnate in [m]')

    # Plot y-coordinates of actual and planned path
    steer_df2.plot(
        ax = ax2,
        x = steer_df.loc[:, 'Iteration'],
        y = [
            steer_df.loc[:, 'Actual Y-Position'],
            steer_df.loc[:, 'Lookahead Y-Waypoint'],
            steer_df.loc[:, 'Closest Y-Waypoint'],
        ],
        kind = 'line',
        label = [
            'actual y-position',
            'lookahead y-waypoint',
            'closest y-waypoint',
        ]
    )
    ax2.set_title('Y-coordinates of actual and planned trajectory')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Y-cooridnate in [m]')

    # Plot distances between actual position and planned path waypoints
    steer_df2.plot(
        ax = ax3,
        x = steer_df.loc[:, 'Iteration'],
        y = [
            steer_df.loc[:, 'Distance to Lookahead Waypoint'],
            steer_df.loc[:, 'Distance to Closest Waypoint'],
        ],
        kind = 'line',
        label = [
            'distance to lookahead waypoint',
            'distance to closest waypoint',
        ]
    )
    ax3.set_title('Distances between actual position and planned path waypoints')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Distance in [m]')

    fig.suptitle('Path coordinates of actual and planned trajectories')
    plt.show()


def plot_trajectories(steer_df, n_rows):
    """Plot recorded actual and planned trajectories."""

    # Define subplot layout
    fig, ax = plt.subplots(nrows=1, ncols=1)

    # Get first n_rows from steering dataframe
    steer_df2 = steer_df[:n_rows]

    # Plot actual trajectory
    steer_df2.plot(
        ax = ax,
        x = steer_df.loc[:, 'Actual X-Position'],
        y = [
            steer_df.loc[:, 'Actual Y-Position'],
        ],
        kind = 'line',
        label = [
            'actual trajectory',
        ],
        color='black'
    )

    # Plot planned trajectory w.r.t. to lookahead waypoint
    steer_df2.plot(
        ax = ax,
        x = steer_df.loc[:, 'Lookahead X-Waypoint'],
        y = [
            steer_df.loc[:, 'Lookahead Y-Waypoint'],
        ],
        kind = 'line',
        label = [
            'planned trajectory (lookahead waypoint)',
        ],
        color='blue'
    )

    # Plot planned trajectory w.r.t. to closest waypoint
    steer_df2.plot(
        ax = ax,
        x = steer_df.loc[:, 'Closest X-Waypoint'],
        y = [
            steer_df.loc[:, 'Closest Y-Waypoint'],
        ],
        kind = 'line',
        label = [
            'planned trajectory (closest waypoint)',
        ],
        color='green'
    )
    ax.set_xlabel('X-coordinate [m]')
    ax.set_ylabel('Y-cooridnate [m]')
    fig.suptitle('Actual and planned trajectories')
    plt.show()

    
def main():
    """Read and plot recorded lateral and longitudinal control data."""
    steer_df = read_steer_data()
    throttle_df = read_throttle_data()
    n_rows = -1 #2000
    plot_steer_data(steer_df, n_rows)
    plot_throttle_data(throttle_df, n_rows)
    plot_throttle_data(steer_df, n_rows)
    plot_trajectories(steer_df, n_rows)


if __name__ == '__main__':
    main()
