import sys
import argparse
import serial
import re
import logging

import numpy as np
import pandas as pd

logging.info("I know it's weird, but it's recommended to run under superuser privilages.")

parser = argparse.ArgumentParser()

parser.add_argument('-d', '--device', type=str, required=True)

# Headers and datatypes for pandas dataframe
header_raw = (r'^ID;Timestamp;GyroscopeX;GyroscopeY;GyroscopeZ;AccelerometerX;AccelerometerY;' \
              'AccelerometerZ;Barometer;Thermometer;ThermometerStupido;Voltage$')

headers = [
    "ID", "Timestamp", "GyroscopeX", "GyroscopeY", "GyroscopeZ", "AccelerometerX", "AccelerometerY",
    "AccelerometerZ", "Barometer", "Thermometer", "ThermometerStupido", "Voltage",
]

datatypes = [int, int, float, float, float, float, float, float, int, float, float, float]

def plot_latest_data(df):
    print(df.iloc[-1])

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