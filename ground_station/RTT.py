import argparse
import serial
import re
import logging
from datetime import datetime

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

parser = argparse.ArgumentParser()

parser.add_argument('-d', '--device', type=str, required=True)
parser.add_argument('-s', '--seconds', type=int, default=6, help='Set the time window for the plot in seconds')
parser.add_argument('-p', '--pressure', type=float, default=1013.25, help='Set pressure according to current location')

# Headers and datatypes for pandas dataframe
header_raw = (r'^ID;Timestamp;GyroscopeX;GyroscopeY;GyroscopeZ;AccelerometerX;AccelerometerY;' \
              'AccelerometerZ;Barometer;Thermometer;ThermometerStupido;Voltage$')

headers = [
    "ID", "Timestamp", "GyroscopeX", "GyroscopeY", "GyroscopeZ", "AccelerometerX", "AccelerometerY",
    "AccelerometerZ", "Barometer", "Thermometer", "ThermometerStupido", "Voltage",
]

datatypes = [int, int, float, float, float, float, float, float, int, float, float, float]

# Initialize the plot
plt.ion()
fig, axs = plt.subplots(3, 3, figsize=(16, 8))  # 3x3 subplot layout

def plot_latest_data(df, window_size=10):
    # Assuming 'Timestamp' is in seconds and sorted
    latest_time = df['Timestamp'].max()
    window_start_time = latest_time - window_size  # 10 seconds window

    # Filter dataframe for the last 10 seconds
    df_window = df[df['Timestamp'] > window_start_time]
    
    
    # Clear previous plots
    for ax in axs.flatten():
        ax.clear()

    # Plot data for each sensor on its own subplot
    sensor_data = ['AccelerometerX', 'AccelerometerY', 'AccelerometerZ',
                   'GyroscopeX', 'GyroscopeY', 'GyroscopeZ',
                   'Voltage', 'Barometer', 'Thermometer']

    for i, sensor in enumerate(sensor_data):
        row = i // 3
        col = i % 3
        axs[row, col].plot(df_window['Timestamp'], df_window[sensor], label=sensor)
        axs[row, col].set_title(sensor)
        axs[row, col].grid(True)

    # Formatting
    for ax in axs[-1, :]:  # Set x-labels for the bottom row only
        ax.set_xlabel('Time (s)')
    for ax in axs[:, 0]:  # Set y-labels for the first column only
        ax.set_ylabel('Value')
    for ax in axs.flatten():
        ax.legend()

    fig.tight_layout()  # Adjust subplots to fit into the figure area
    plt.draw()
    plt.pause(0.001)  # Pause to allow the plot to update

def dump_dataframe(df):
    '''Dump current content of dataframe to a file in PWD/LOG_timestamp_now.csv'''
    df.to_csv(f'LOG_{datetime.now().isoformat()}.csv', index=False)

def is_csv_header(line):
    '''Match with our CSV header'''
    return bool(re.match(header_raw, line))

def is_csv_row(line):
    '''Is numerical CSV row?'''
    return bool(re.match(r'^-?\d+(\.\d+)?(;-?\d+(\.\d+)?)*$', line))

def append_csv_row(row, df):
    '''Add CSV row to our current dataframe'''
    values = row.split(';')
    data = {header: dt(val) for header, dt, val in zip(headers, datatypes, values)}
    # Normalize the timeticks
    data['Timestamp'] = data['Timestamp'] / 1000000
    return pd.concat([df, init_dataframe(data=data)], ignore_index=True)

def init_dataframe(data=None):
    if data is None:
        return pd.DataFrame(columns=headers)
    else:
        return pd.DataFrame(data, columns=headers, index=[0]).astype(dict(zip(headers, datatypes)))

def init_line_serial(device):
    '''Initialize the serial communication'''
    # Serial communication setup
    return serial.Serial(device, 115200, timeout=1)

def read_line_serial(ser, df, activated):
    '''Let's read a single line from the I2C'''
    try:
        line = ser.readline()
        line = line.decode('utf-8').strip()

        if is_csv_header(line):
            if activated:
                dump_dataframe(df)
                df = init_dataframe()
            activated = True
        elif is_csv_row(line) and activated:
            df = append_csv_row(line, df)
            plot_latest_data(df)
        else: 
            logging.debug("Unrecognized line format: %s", line)

    except UnicodeDecodeError:
        logging.warning("Failed to decode line: %s", line)
    except Exception as e:
        logging.error("Unexpected error processing line: %s", repr(e))

    return df, activated


if __name__ == "__main__":
    
    args = parser.parse_args()

    activated = False
    ser = init_line_serial(args.device)

    # Initialize DataFrame
    df = init_dataframe()

    while True:

        df, activated = read_line_serial(ser, df, activated)