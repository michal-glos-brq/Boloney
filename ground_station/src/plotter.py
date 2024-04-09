import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import re


HEADER = 69

# # DataFrame setup
headers = ["ID", "Timestamp", "OrientationX", "OrientationY", "OrientationZ", "PositionX", "PositionY",
           "PositionZ", "Barometer", "Thermometer", "ThermometerStupido", "Voltage"]
header_pattern = re.compile(
    (r'^ID;Timestamp;OrientationX;OrientationY;OrientationZ;PositionX;PositionY;PositionZ;'
     r'Barometer;Thermometer;ThermometerStupido;Voltage$')
)
# Pandas is not great at guessing datatypes, let's define those explicitly
datatypes = [int, float, float, float, float, float, float, float, float, float, float, float]

sensor_data = ['PositionX', 'PositionY', 'PositionZ',
               'OrientationX', 'OrientationY', 'OrientationZ',
               'Voltage', 'Barometer', 'Thermometer']
sensor_colors = {
    'PositionX': 'r', 'PositionY': 'g', 'PositionZ': 'b',
    'OrientationX': 'c', 'OrientationY': 'm', 'OrientationZ': 'y',
    'Voltage': 'k', 'Barometer': 'orange', 'Thermometer': 'purple'
}




# Initialize the plot
plt.ion()
fig, axs = plt.subplots(3, 3, figsize=(16, 8))  # 3x3 subplot layout
fig.subplots_adjust(hspace=0.4, wspace=0.4)  # Adjust space between plots
# Flatten axs for easier iteration
axs_flat = axs.flatten()

def setup_subplot_axes():
    # Setup subplot axes just once if their configuration doesn't depend on the data
    for i, ax in enumerate(axs_flat):
        if i < len(sensor_data):
            sensor = sensor_data[i]
            ax.set_title(sensor)
            ax.grid(True)
            ax.legend(loc='upper left')
            ax.xaxis.set_major_locator(MaxNLocator(integer=True))
            # Set labels for the bottom row and first column only
            if i // 3 == 2: ax.set_xlabel('Time (s)')
            if i % 3 == 0: ax.set_ylabel('Value')

setup_subplot_axes()  # Call it once to setup

def plot_latest_data(df, window_size=10):
    latest_time = df['Timestamp'].max()
    window_start_time = latest_time - window_size

    # Ensure the DataFrame is sorted by 'Timestamp'. Pandas sorting is efficient and optimized.
    df_sorted = df.sort_values('Timestamp')
    
    df_window = df_sorted[df_sorted['Timestamp'] > window_start_time]

    # Clear previous plots
    for ax in axs_flat:
        ax.clear()

    y_limits = {}
    for sensor in sensor_data:
        if sensor in df_window:
            min_val, max_val = df_window[sensor].min(), df_window[sensor].max()
            y_limits[sensor] = [min_val, max_val]

    for i, sensor in enumerate(sensor_data):
        if i < len(axs_flat) and sensor in df_window:
            ax = axs_flat[i]
            color = sensor_colors[sensor]
            ax.plot(df_window['Timestamp'], df_window[sensor], label=sensor, color=color)
            ax.set_xlim(window_start_time, latest_time)
            if sensor in y_limits:
                ax.set_ylim(y_limits[sensor])

    plt.draw()
    plt.pause(0.001)  # Pause to allow the plot to update
    
    
    
    



# plt.ion()
# fig = plt.figure(figsize=(16, 8))

# # Create subplots for Position (3D trajectory) and Orientation (Polar plots) + other sensors
# ax_position = fig.add_subplot(2, 4, 1, projection='3d')
# ax_orientation_x = fig.add_subplot(2, 4, 2, polar=True)
# ax_orientation_y = fig.add_subplot(2, 4, 3, polar=True)
# ax_orientation_z = fig.add_subplot(2, 4, 4, polar=True)
# axs_sensors = [fig.add_subplot(2, 4, 5), fig.add_subplot(2, 4, 6), fig.add_subplot(2, 4, 7), fig.add_subplot(2, 4, 8)]

# # Setup for Position and Orientation plots
# ax_position.set_title('Position Trajectory')
# ax_position.set_xlabel('X')
# ax_position.set_ylabel('Y')
# ax_position.set_zlabel('Z')
# for ax, orientation in zip([ax_orientation_x, ax_orientation_y, ax_orientation_z], ['OrientationX', 'OrientationY', 'OrientationZ']):
#     ax.set_title(f'{orientation} Polar')
#     ax.set_theta_zero_location('N')
#     ax.set_theta_direction(-1)

# # Setup for other sensor plots
# sensor_titles = ['Voltage', 'Barometer', 'Thermometer', 'ThermometerStupido']
# for ax, title in zip(axs_sensors, sensor_titles):
#     ax.set_title(title)
#     ax.grid(True, which='both', axis='y')
#     ax.set_xlabel('Time (s)')
#     ax.set_ylabel('Value')

# def plot_latest_data(df, window_size=10):
#     latest_time = df['Timestamp'].max()
#     window_start_time = latest_time - window_size
#     df_window = df[(df['Timestamp'] > window_start_time)].sort_values('Timestamp')

#     # Plot Position as a trajectory
#     ax_position.clear()
#     ax_position.plot(df_window['PositionX'], df_window['PositionY'], df_window['PositionZ'], label='Trajectory')
#     ax_position.legend()

#     # Plot Orientation in polar coordinates
#     for ax, orientation in zip([ax_orientation_x, ax_orientation_y, ax_orientation_z], ['OrientationX', 'OrientationY', 'OrientationZ']):
#         ax.clear()
#         theta = np.radians(df_window[orientation])
#         r = np.linspace(0, 1, len(df_window))
#         ax.plot(theta, r)
#         ax.set_title(f'{orientation} Polar')

#     # Plot other sensor data
#     sensor_columns = ['Voltage', 'Barometer', 'Thermometer', 'ThermometerStupido']
#     for ax, sensor in zip(axs_sensors, sensor_columns):
#         ax.clear()
#         ax.plot(df_window['Timestamp'], df_window[sensor], label=sensor)
#         ax.grid(True, which='both', axis='y')
#         ax.legend()

#     plt.tight_layout()
#     plt.draw()
#     plt.pause(0.001)



# # This file contains plotting logic and chart initialization

    
    
# def draw_orientation_arrow(ax, angle, title):
#     ax.clear()
#     ax.set_xlim(-1, 1)
#     ax.set_ylim(-1, 1)
#     ax.axis('off')

#     # The arrow represents the direction of the orientation
#     arrow = plt.Arrow(0, 0, 0.8*np.sin(np.radians(angle)), 0.8*np.cos(np.radians(angle)), width=0.1)
#     ax.add_patch(arrow)

#     # Draw a circle to represent the possible range of motion
#     circle = plt.Circle((0, 0), 1, fill=False, color='black', linestyle='--')
#     ax.add_patch(circle)

#     # Set the title of the subplot
#     ax.set_title(title)

# # Updated function to plot latest data
# def plot_latest_data(df, window_size=10):
#     # Calculate the time window to plot
#     latest_time = df['Timestamp'].max()
#     window_start_time = latest_time - window_size
    
#     # Filter dataframe for the last 10 seconds
#     df_window = df[df['Timestamp'] >= window_start_time]

#     # Plot trajectories in the first row
#     for i, axis in enumerate(['PositionX', 'PositionY', 'PositionZ']):
#         axs[0, i].plot(df_window['Timestamp'], df_window[axis], label=axis, color=sensor_colors[axis])
#         axs[0, i].set_title(f'Trajectory of {axis}')
#         axs[0, i].grid(True)
#         axs[0, i].set_xlim(window_start_time, latest_time)
#         axs[0, i].xaxis.set_major_locator(MaxNLocator(integer=True))
#         axs[0, i].legend()

#     # Plot orientation arrows in the second row
#     for i, axis in enumerate(['OrientationX', 'OrientationY', 'OrientationZ']):
#         draw_orientation_arrow(axs[1, i], df_window[axis].iloc[-1], axis)
    
#     # Plot time series data in the third row
#     for i, sensor in enumerate(['Voltage', 'Barometer', 'Thermometer']):
#         axs[2, i].plot(df_window['Timestamp'], df_window[sensor], label=sensor, color=sensor_colors[sensor])
#         axs[2, i].set_title(sensor)
#         axs[2, i].grid(True)
#         axs[2, i].set_xlim(window_start_time, latest_time)
#         axs[2, i].xaxis.set_major_locator(MaxNLocator(integer=True))
#         axs[2, i].legend()

#     # Set labels for the last column and row
#     for ax in axs[-1, :]:  # x-labels for the bottom row
#         ax.set_xlabel('Time (s)')
#     for ax in axs[:, 0]:  # y-labels for the first column
#         ax.set_ylabel('Value')

#     plt.tight_layout()
#     plt.draw()
#     plt.pause(0.001)