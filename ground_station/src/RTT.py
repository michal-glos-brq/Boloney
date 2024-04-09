#! /usr/bin/env python3

import argparse
import serial
import re

from multiprocessing import Process, Queue

import logging
logging.basicConfig(level=logging.DEBUG)
logging.getLogger('matplotlib').setLevel(logging.WARNING)

from datetime import datetime
import pandas as pd

import plotter


MAX_MSGS_PER_UPDATE = 1


# Setup the argument parser
parser = argparse.ArgumentParser()
parser.add_argument('-d', '--device', type=str, required=True)
parser.add_argument('-s', '--seconds', type=int, default=10, help='Set the time window for the plot in seconds')
parser.add_argument('-p', '--pressure', type=float, default=1013.25, help='Set pressure according to current location')


def reading_process(queue):
    '''Read serial port and give the lines and headers into Queue'''
    activated = False
    
    msg_counter = 0
    
    while True:
        try:
            # Read a single line
            line = ''
            while True:
                char = ser.read(1).decode('utf-8')  # Read one character at a time
                if char == '\n':  # Check if the character is a newline, indicating the end of a line
                    break  # Exit the loop, as we've finished reading one line
                line += char  # Accumulate the character into the line

            # Strip and process to plot
            line = line.strip()
            # If header was read, alert other process
            if bool(re.match(plotter.header_pattern, line)):
                logging.warning("Header received!")
                activated = True
                queue.put(plotter.HEADER)
            # If line was read, put it into queue
            elif bool(re.match(r'^-?\d+(\.\d+)?(;-?\d+(\.\d+)?)*$', line)) and activated:
                msg_counter += 1
                values = line.split(';')
                data = {header: dt(val) for header, dt, val in zip(plotter.headers, plotter.datatypes, values)}
                data['Timestamp'] = data['Timestamp'] / 1000000.
                queue.put(data)
                logging.info(f"Message rcv: {data['ID']}")
                # Just make sure the stream gets disrupted so it could be plotted
                if msg_counter >= MAX_MSGS_PER_UPDATE:
                    msg_counter = 0
                    queue.put(None)
            else: 
                logging.debug("Unrecognized line: %s", line)

        except UnicodeDecodeError:
            # This means restart - initial message could not be decoded in UTF8
            queue.put(None)
        except serial.SerialException as e:
            logging.info("Exiting...")
            ser.close()
            exit()
        except Exception as e:
            logging.error("Unexpected error processing line: %s", repr(e))

def plotting_process(queue):
    '''Read from queue and make plots'''
    activated = False
    while True:
        new_data = False
        # Read the whole queue
        data = []
        while True:
            dato = queue.get()
            if dato is None:
                break
            data.append(dato)
        
        
        for dato in data:
            if dato is None:
                continue
            if dato == plotter.HEADER:
                # If activated before, save CSV
                if activated:
                    df.to_csv(f'./CSV/{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.csv', index=False)
                # Activate and create new DF
                df = pd.DataFrame(columns=plotter.headers)
                activated = True
                continue
            else:
                df.loc[len(df)] = dato
                new_data = True
        
        if new_data:        
            plotter.plot_latest_data(df, args.seconds)
        


if __name__ == "__main__":
    # Parse the CLI parameters
    args = parser.parse_args()
 
    # Connect to the ground station
    ser = serial.Serial(args.device, 115200, timeout=1)
    # Initialize DataFrame
    df = pd.DataFrame(columns=plotter.headers)

    queue = Queue()
    
    reader = Process(target=reading_process, args=(queue,))
    reader.start()
    
    # This is done in main thread
    plotting_process(queue)

    reader.join()
 
    
