import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np
import time
import logging


# Assuming these are your global variables
sensor_colors = {
    'PositionX': 'r', 'PositionY': 'g', 'PositionZ': 'b',
    'VelocityX': 'c', 'VelocityY': 'm', 'VelocityZ': 'y',
    'OrientationX': 'c', 'OrientationY': 'm', 'OrientationZ': 'y',
    'AngularVelocityX': 'r', 'AngularVelocityY': 'g', 'AngularVelocityZ': 'b',
    'Voltage': 'k', 'Barometer': 'orange', 'Thermometer': 'purple',
}

# Sensor units for labels
sensor_units = {
    'PositionX': 'm', 'PositionY': 'm', 'PositionZ': 'm',
    'VelocityX': 'm/s', 'VelocityY': 'm/s', 'VelocityZ': 'm/s',
    'OrientationX': 'degrees', 'OrientationY': 'degrees', 'OrientationZ': 'degrees',
    'AngularVelocityX': 'deg/s', 'AngularVelocityY': 'deg/s', 'AngularVelocityZ': 'deg/s',
    'Voltage': 'V', 'Barometer': 'Pa', 'Thermometer': '°C'
}

# Placeholder for Y-axis limits, adjust based on your data
# Example: {'PositionX': (-10, 10), 'PositionY': (-10, 10), ...}
sensor_limits = {
    'PositionX': (-1000, 1000), 'PositionY': (-1000, 1000), 'PositionZ': (-1000, 1000),
    'VelocityX': (-10, 10), 'VelocityY': (-10, 10), 'VelocityZ': (-10, 10),
    'OrientationX': (0, 360), 'OrientationY': (0, 360), 'OrientationZ': (0, 360),
    'AngularVelocityX': (-180, 180), 'AngularVelocityY': (-180, 180), 'AngularVelocityZ': (-180, 180),
    'Voltage': (0, 5), 'Barometer': (95000, 105000), 'Thermometer': (-20, 80)
}


plt.ion()
fig, axs = plt.subplots(5, 3, figsize=(16, 12))
fig.subplots_adjust(hspace=0.4, wspace=0.4)
plt.tight_layout()

lines = {}  # Dictionary to hold the Line2D objects for each sensor

def draw_orientation_arrow(ax, angle, title, sensor):
    if sensor not in lines or 'arrow' not in lines[sensor]:
        # Clear the axis for a new drawing if it's the first time or if the arrow doesn't exist
        ax.clear()
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        ax.set_aspect('equal')  # Ensure the aspect ratio is equal to maintain the circle shape
        ax.axis('off')

        # Draw the arrow representing the direction of the orientation
        arrow = plt.Arrow(0, 0, 0.8 * np.sin(np.radians(angle)), 0.8 * np.cos(np.radians(angle)), width=0.1)
        lines[sensor] = {'arrow': ax.add_patch(arrow)}

        # Draw a circle to represent the possible range of motion
        circle = plt.Circle((0, 0), 1, fill=False, color='black', linestyle='--')
        ax.add_patch(circle)

        ax.set_title(title)
    else:
        # Remove the existing arrow and draw a new one with the updated orientation
        lines[sensor]['arrow'].remove()
        arrow = plt.Arrow(0, 0, 0.8 * np.sin(np.radians(angle)), 0.8 * np.cos(np.radians(angle)), width=0.1)
        lines[sensor]['arrow'] = ax.add_patch(arrow)

    
def plot_latest_data(df, window_size=10, partial_render=False):
    start = time.time()
    latest_time = df['Timestamp'].max()
    window_start_time = latest_time - window_size
    df_window = df[df['Timestamp'] >= window_start_time]

    total_seconds = latest_time - window_start_time
    tick_spacing = max(int(total_seconds / 5), 1)

    for i, axis in enumerate(['PositionX', 'PositionY', 'PositionZ', 'OrientationX', 'OrientationY', 'OrientationZ', 'VelocityX', 'VelocityY', 'VelocityZ', 'AngularVelocityX', 'AngularVelocityY', 'AngularVelocityZ', 'Voltage', 'Barometer', 'Thermometer']):
        ax = axs[i // 3, i % 3]

        if axis.startswith('Orientation'):  # Special case for orientation arrows
            draw_orientation_arrow(ax, df_window[axis].iloc[-1], f'{axis} ({sensor_units[axis]})', axis)
            continue

        if axis not in lines:
            line, = ax.plot(df_window['Timestamp'], df_window[axis], label=axis, color=sensor_colors[axis])
            lines[axis] = line
        else:
            lines[axis].set_data(df_window['Timestamp'], df_window[axis])

        ax.set_ylim(sensor_limits[axis])
        ax.set_xlim(window_start_time, latest_time)
        ax.xaxis.set_major_locator(MaxNLocator(nbins=5, steps=[1, 2, 5, 10]))
        if not partial_render:
            ax.set_title(f'{axis} ({sensor_units[axis]})')
            ax.grid(True)

    for ax in axs[-1, :]:
        ax.set_xlabel('Time (s)')
    for ax_row, sensor in zip(axs, ['PositionX', 'OrientationX', 'VelocityX', 'AngularVelocityX', 'Voltage']):
        ax_row[0].set_ylabel(f'Value ({sensor_units[sensor]})')

    plt.tight_layout()

    if not partial_render:
        plt.draw()
        plt.pause(0.1)
        fig.canvas.flush_events()

    logging.info(f"Plotting took {time.time() - start:.2f} seconds")