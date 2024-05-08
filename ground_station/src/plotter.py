import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import time
import logging

import traceback

# Global airplane models for each axis
plane_models = {
    "OrientationY": {
        "nosecone": np.array([[-0.15, 0.8], [0.15, 0.8], [0, 1], [-0.15, 0.8]]),
        "body": np.array([[-0.15, -0.8], [0.15, -0.8], [0.15, 0.8], [-0.15, 0.8], [-0.15, -0.8]]),
        "wings": np.array([[-1, 0.1], [1, 0.1], [0, 0.5], [-1, 0.1]]),
        "tail": np.array([[-0.5, -0.7], [0.5, -0.7], [0, -0.4], [-0.5, -0.7]]),
    },
    "OrientationZ": {
        "body": np.array([[-0.15, 0.15], [0.15, 0.15], [0.15, -0.15], [-0.15, -0.15], [-0.15, 0.15]]),
        "wings": np.array([[-1, 0.13], [1, 0.13], [1, 0.08], [-1, 0.08]]),
        "tail": np.array([[-0.03, 0.15], [0.03, 0.15], [0, 0.5], [-0.03, 0.15]]),
    },
    "OrientationX": {
        "nosecone": np.array([[-0.15, 0.8], [0.15, 0.8], [0, 1], [-0.15, 0.8]]),
        "body": np.array([[-0.15, -0.8], [0.15, -0.8], [0.15, 0.8], [-0.15, 0.8], [-0.15, -0.8]]),
        "tail": np.array([[0.15, -0.55], [0.15, -0.75], [0.55, -0.79], [0.15, -0.55]]),
    },
}


pitch_yaw_roll = {"X": "Pitch", "Y": "Yaw", "Z": "Roll"}

metrics = [
    "OrientationX",
    "OrientationY",
    "OrientationZ",
    "AngularVelocityX",
    "AngularVelocityY",
    "AngularVelocityZ",
    "PositionX",
    "PositionY",
    "PositionZ",
    "VelocityX",
    "VelocityY",
    "VelocityZ",
    "AccelerationX",
    "AccelerationY",
    "AccelerationZ",
    "Barometer",
    "Thermometer",
    "Voltage",
]

y_axis_ranges = {}

colors = plt.cm.jet(np.linspace(0, 1, len(metrics)))  # Generate distinct colors

def init_plots(simplified=False):
    plt.ion()
    if simplified:
        # Adjusted subplot configuration to be wider
        fig, axs = plt.subplots(3, 6, figsize=(14, 7))  # 3 rows, 5 cols, wider than high
        fig.subplots_adjust(hspace=0.35, wspace=0.5)
    else:
        fig, axs = plt.subplots(2, 3, figsize=(14, 7))  # 2 rows, 3 cols
        fig.subplots_adjust(hspace=0.4, wspace=0.25)
    return fig, axs


def draw_orientation(ax, angle, axis):
    ax.clear()
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_aspect("equal")
    ax.axis("off")

    rotation = pitch_yaw_roll[axis.split("Orientation")[1]]

    model = plane_models[axis]
    offset = np.array([0, 0])

    if rotation == "Pitch":
        rotation_matrix = np.array(
            [
                [np.cos(np.radians(angle - 90)), -np.sin(np.radians(angle - 90))],
                [np.sin(np.radians(angle - 90)), np.cos(np.radians(angle - 90))],
            ]
        )
    elif rotation == "Yaw":
        rotation_matrix = np.array(
            [
                [np.cos(np.radians(-angle)), -np.sin(np.radians(-angle))],
                [np.sin(np.radians(-angle)), np.cos(np.radians(-angle))],
            ]
        )
    else:
        rotation_matrix = np.array(
            [
                [np.cos(np.radians(angle)), -np.sin(np.radians(angle))],
                [np.sin(np.radians(angle)), np.cos(np.radians(angle))],
            ]
        )

    for part_name, part in model.items():
        rotated_part = np.dot(part * 2, rotation_matrix)
        polygon = patches.Polygon(rotated_part + offset, closed=True, color="blue")
        ax.add_patch(polygon)

    for r in np.linspace(0.4, 2.8, 7):
        circle = plt.Circle(offset, r, fill=False, color="gray", linestyle="--", linewidth=0.5)
        ax.add_patch(circle)

    for _angle in np.linspace(0, 360, 12):
        ax.plot(
            [offset[0], offset[0] + 3 * np.sin(np.radians(_angle))],
            [offset[1], offset[1] + 3 * np.cos(np.radians(_angle))],
            "gray",
            linestyle="--",
            linewidth=0.5,
        )

    ax.text(-2.8, -2.8, f"{angle}°", ha="left", va="bottom", fontsize=12)
    ax.set_title(f"{rotation}", fontsize=10, pad=-10)



line_objects = {}


def plot_all_data(df, axs, fig, window_size=10):
    start = time.time()

    # Determine the latest timestamp to create a sliding window of the last 'window_size' seconds
    latest_time = df["Timestamp"].max()
    window_start_time = latest_time - window_size
    df_window = df[df["Timestamp"] >= window_start_time]
    df_window = df_window.sort_values("Timestamp")
    axs = np.array(axs.flatten()).transpose()

    for i, metric in enumerate(metrics):
        ax = axs[i]
        
        # If line objects for this metric don't exist, create them.
        if metric not in line_objects:
            (line,) = ax.plot(df_window["Timestamp"], df_window[metric], color=colors[i], label=metric)
            line_objects[metric] = line
            ax.set_title(metric)  # Set title only once
        else:
            # Update the data for existing line objects
            line_objects[metric].set_data(df_window["Timestamp"], df_window[metric])

        try:
            # Dynamically set Y-axis limits based on current data
            ax.set_ylim([df_window[metric].min(), df_window[metric].max()])
        except Exception as e:
            logging.error(f"Error setting y-axis limits for {metric}: {e}")
            # Traceback
            logging.debug(traceback.format_exc())

        # Adjust the x-axis limits to the sliding window
        ax.set_xlim(window_start_time, latest_time)

    plt.draw()
    fig.canvas.flush_events()
    plt.pause(0.01)  # Pause briefly to update the plot
    logging.info(f"Plotting took {time.time() - start:.2f} seconds")


def plot_latest_data(df, axs, fig, window_size=10, ref_pressure=101325):
    start = time.time()
    latest_time = df["Timestamp"].max()
    window_start_time = latest_time - window_size
    df_window = df[df["Timestamp"] >= window_start_time]


    # Text display
    ax_text = axs[0, 2]
    ax_text.clear()
    ax_text.axis("off")
    distance = np.sqrt(df_window["PositionX"].iloc[-1] ** 2 + df_window["PositionY"].iloc[-1] ** 2)
    temperature = df_window["Thermometer"].iloc[-1]
    voltage = df_window["Voltage"].iloc[-1]

    speed = np.sqrt(
        df_window["VelocityX"].iloc[-1] ** 2
        + df_window["VelocityY"].iloc[-1] ** 2
        + df_window["VelocityZ"].iloc[-1] ** 2
    )


    # Calculating total acceleration and angular acceleration
    total_acceleration = np.sqrt(df_window["AccelerationX"].iloc[-1]**2 +
                                 df_window["AccelerationY"].iloc[-1]**2 +
                                 df_window["AccelerationZ"].iloc[-1]**2)

    total_angular_acceleration = np.sqrt(df_window["AngularVelocityX"].iloc[-1]**2 +
                                         df_window["AngularVelocityY"].iloc[-1]**2 +
                                         df_window["AngularVelocityZ"].iloc[-1]**2)



    ax_text.text(0.5, 0.7, f"Distance: {distance:.2f} m", ha="center", fontsize=12, fontweight="bold")
    ax_text.text(0.5, 0.6, f"Speed: {speed:.2f} m/s", ha="center", fontsize=12, fontweight="bold")
    ax_text.text(0.5, 0.5, f"Acceleration: {total_acceleration:.2f} m^2/s", ha="center", fontsize=12, fontweight="bold")
    ax_text.text(0.5, 0.4, f"Angular Accel.: {total_angular_acceleration:.2f} degrees^2/s", ha="center", fontsize=12, fontweight="bold")
    ax_text.text(0.5, 0.3, f"Temperature: {temperature:.2f} °C", ha="center", fontsize=12, fontweight="bold")
    ax_text.text(0.5, 0.2, f"Voltage: {voltage:.2f} V", ha="center", fontsize=12, fontweight="bold")


    # Plot X and Y position in a single plot
    if 'trajectory' not in line_objects:
        # Initialize plot
        ax_xy = axs[0, 0]
        ax_xy.set_xlim(-1000, 1000)
        ax_xy.set_ylim(-1000, 1000)
        ax_xy.set_xlabel("Position X (m)")
        ax_xy.set_ylabel("Position Y (m)")
        ax_xy.set_title("Position X vs. Y", fontsize=10)
        ax_xy.grid(True)
        
        # Create line and scatter objects
        (line,) = ax_xy.plot([], [], label="Trajectory", color="gray")
        scatter = ax_xy.scatter([], [], color="red", s=100)
        
        line_objects['trajectory'] = (line, scatter)
    else:
        line, scatter = line_objects['trajectory']
        line.set_data(df_window["PositionX"], df_window["PositionY"])
        scatter.set_offsets([df_window["PositionX"].iloc[-1], df_window["PositionY"].iloc[-1]])

    # Check if 'altitude' plot exists, if not create it
    if 'altitude' not in line_objects:
        ax_altitude = axs[0, 1]
        ax_altitude.set_ylim(-100, 500)  # Set reasonable default limits
        ax_altitude.set_xlim(window_start_time, latest_time)
        ax_altitude.set_title("Altitude Estimations", fontsize=10)
        ax_altitude.grid(True)

        # Plot initializing
        line1, = ax_altitude.plot([], [], label="Altitude from Z", color="blue")
        line2, = ax_altitude.plot([], [], label="Altitude from Barometer", color="orange")
        ax_altitude.legend()

        line_objects['altitude'] = (line1, line2)
    else:
        line1, line2 = line_objects['altitude']
        
        # Convert barometer data to altitude
        altitude_from_barometer = (
            1 - (df_window["Barometer"] / ref_pressure) ** (1 / 5.25588)
        ) * 44330.77  # Update formula for altitude

        # Update data for plots
        line1.set_data(df_window["Timestamp"], df_window["PositionZ"])
        line2.set_data(df_window["Timestamp"], altitude_from_barometer)

        # Update limits dynamically based on new data
        ax_altitude = line1.axes
        ax_altitude.set_xlim(window_start_time, latest_time)
        ax_altitude.set_ylim(min(-100, (df_window["PositionZ"].min() - 25), (altitude_from_barometer.min()-25)),
                             max(df_window["PositionZ"].max()+25, 500, (altitude_from_barometer.max()+25)))

        # Will be added also here:
        ax_text.text(0.5, 0.85, f"Altitude: {altitude_from_barometer.iloc[-1]:.2f} m", ha="center", fontsize=12, fontweight="bold")

    
    # Orientation plots
    for i, axis in enumerate(["OrientationX", "OrientationY", "OrientationZ"]):
        ax = axs[1, i % 3]
        draw_orientation(ax, df_window[axis].iloc[-1], axis)

    plt.draw()
    fig.canvas.flush_events()
    plt.pause(0.01)

    logging.info(f"Plotting took {time.time() - start:.2f} seconds")
