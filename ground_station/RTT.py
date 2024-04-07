#! /usr/bin/env python3

import argparse
import serial
import re
import logging
from datetime import datetime

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

# Setup the argument parser
parser = argparse.ArgumentParser()
parser.add_argument('-d', '--device', type=str, required=True)
parser.add_argument('-s', '--seconds', type=int, default=10, help='Set the time window for the plot in seconds')
parser.add_argument('-p', '--pressure', type=float, default=1013.25, help='Set pressure according to current location')

# DataFrame setup
headers = ["ID", "Timestamp", "OrientationX", "OrientationY", "OrientationZ", "PositionX", "PositionY",
           "PositionZ", "Barometer", "Thermometer", "ThermometerStupido", "Voltage"]
header_pattern = re.compile(
    (r'^ID;Timestamp;OrientationX;OrientationY;OrientationZ;PositionX;PositionY;PositionZ;'
     r'Barometer;Thermometer;ThermometerStupido;Voltage$')
)
# Pandas is not great at guessing datatypes, let's define those explicitly
datatypes = [int, float, float, float, float, float, float, float, float, float, float, float]

# Initialize the plot
plt.ion()
fig, axs = plt.subplots(3, 3, figsize=(16, 8))  # 3x3 subplot layout
fig.subplots_adjust(hspace=0.4, wspace=0.4)  # Adjust space between plots

# Colors for each sensor type for better distinction
sensor_colors = {
    'PositionX': 'r', 'PositionY': 'g', 'PositionZ': 'b',
    'OrientationX': 'c', 'OrientationY': 'm', 'OrientationZ': 'y',
    'Voltage': 'k', 'Barometer': 'orange', 'Thermometer': 'purple'
}

def plot_latest_data(df, window_size=10):
    '''Improved plotting for readability, aesthetics, and shared Y-axes.'''
    # Assuming 'Timestamp' is in seconds and sorted
    latest_time = df['Timestamp'].max()
    window_start_time = latest_time - window_size  # 10 seconds window

    # Filter dataframe for the last 10 seconds
    df_window = df[df['Timestamp'] > window_start_time]
    df_window = df_window.sort_values('Timestamp')
    
    # Clear previous plots
    for ax in axs.flatten():
        ax.clear()

    # Determine Y-axis limits for shared Y-axis groups (e.g., all accelerometer axes share the same Y-axis limits)
    y_limits = {}
    for sensor in ['PositionX', 'PositionY', 'PositionZ', 'OrientationX', 'OrientationY', 'OrientationZ']:
        min_val = df_window[sensor].min()
        max_val = df_window[sensor].max()
        if sensor.startswith('Accelerometer') or sensor.startswith('Gyroscope'):
            key = sensor[:len('Accelerometer')]  # Group by 'Accelerometer' or 'Gyroscope'
        else:
            key = sensor
        if key not in y_limits:
            y_limits[key] = [min_val, max_val]
        else:
            y_limits[key][0] = min(y_limits[key][0], min_val)
            y_limits[key][1] = max(y_limits[key][1], max_val)

    # Plot data for each sensor on its own subplot
    sensor_data = ['PositionX', 'PositionY', 'PositionZ',
                   'OrientationX', 'OrientationY', 'OrientationZ',
                   'Voltage', 'Barometer', 'Thermometer']

    for i, sensor in enumerate(sensor_data):
        row = i // 3
        col = i % 3
        color = sensor_colors[sensor]
        axs[row, col].plot(df_window['Timestamp'], df_window[sensor], label=sensor, color=color)
        axs[row, col].set_title(sensor)
        axs[row, col].grid(True)
        axs[row, col].set_xlim(window_start_time, latest_time)  # Fixing the x-axis range
        axs[row, col].xaxis.set_major_locator(MaxNLocator(integer=True))  # Integer x-axis values
        # Set shared Y-axis limits for Accelerometers and Gyroscopes
        if sensor.startswith('Accelerometer') or sensor.startswith('Gyroscope'):
            key = sensor[:len('Accelerometer')]
            axs[row, col].set_ylim(y_limits[key])

    # Formatting
    for ax in axs[-1, :]:  # Set x-labels for the bottom row only
        ax.set_xlabel('Time (s)')
    for ax in axs[:, 0]:  # Set y-labels for the first column only
        ax.set_ylabel('Value')
    for ax in axs.flatten():
        ax.legend(loc='upper left')

    fig.tight_layout()  # Adjust subplots to fit into the figure area
    plt.draw()
    plt.pause(0.001)  # Pause to allow the plot to update

def append_csv_row(row, df):
    '''Add CSV row to our current dataframe'''
    values = row.split(';')
    data = {header: dt(val) for header, dt, val in zip(headers, datatypes, values)}
    # Normalize the timeticks
    data['Timestamp'] = data['Timestamp'] / 1000000.
    _df = pd.DataFrame(data, columns=headers, index=[0]).astype(dict(zip(headers, datatypes)))
    return pd.concat([df, _df], ignore_index=True)

def init_line_serial(device):
    '''Initialize the serial communication'''
    # Serial communication setup
    return serial.Serial(device, 115200, timeout=1)

def read_line_serial(ser, df, activated, window_size):
    '''Let's read a single line from the I2C'''
    try:
        line = ser.readline()
        line = line.decode('utf-8').strip()

        if bool(re.match(header_pattern, line)):
            if activated:
                df.to_csv(f'LOG_{datetime.now().isoformat()}.csv', index=False)
                df = pd.DataFrame(columns=headers)
            activated = True
        elif bool(re.match(r'^-?\d+(\.\d+)?(;-?\d+(\.\d+)?)*$', line)) and activated:
            df = append_csv_row(line, df)
            plot_latest_data(df, window_size=window_size)
        else: 
            logging.debug("Unrecognized line format: %s", line)

    except UnicodeDecodeError:
        logging.warning("Failed to decode line: %s", line)
    except Exception as e:
        logging.error("Unexpected error processing line: %s", repr(e))

    return df, activated


if __name__ == "__main__":
    # Parse the CLI parameters
    args = parser.parse_args()
    # Whether data are being collected - activates when CSV header is received (on GS boot)
    # and deacitvate, when receiving un-decode-able line
    activated = False
    # Connect to the ground station
    ser = init_line_serial(args.device)
    # Initialize DataFrame
    df = pd.DataFrame(columns=headers)

    # Start the ethernal loop of reading and parsing
    while True:
        df, activated = read_line_serial(ser, df, activated, args.seconds)