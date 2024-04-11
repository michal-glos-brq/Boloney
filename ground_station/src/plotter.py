import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import time
import logging

# Global airplane models for each axis
plane_models = {
    'OrientationY': {
        'nosecone': np.array([[-0.15, 0.8], [0.15, 0.8], [0, 1], [-0.15, 0.8]]),
        'body': np.array([[-0.15, -0.8], [0.15, -0.8], [0.15, 0.8], [-0.15, 0.8], [-0.15, -0.8]]),
        'wings': np.array([[-1, 0.1], [1, 0.1], [0, 0.5], [-1, 0.1]]),
        'tail': np.array([[-0.5, -0.7], [0.5, -0.7], [0, -0.4], [-0.5, -0.7]]),
    },
    'OrientationZ': {
        'body': np.array([[-0.15, 0.15], [0.15, 0.15], [0.15, -0.15], [-0.15, -0.15], [-0.15, 0.15]]),
        'wings': np.array([[-1, 0.13], [1, 0.13], [1, 0.08], [-1, 0.08]]),
        'tail': np.array([[-0.03, 0.15], [0.03, 0.15], [0, 0.5], [-0.03, 0.15]])
    },
    'OrientationX': {
        'nosecone': np.array([[-0.15, 0.8], [0.15, 0.8], [0, 1], [-0.15, 0.8]]),
        'body': np.array([[-0.15, -0.8], [0.15, -0.8], [0.15, 0.8], [-0.15, 0.8], [-0.15, -0.8]]),
        'tail': np.array([[0.15, -0.55], [0.15, -0.75], [0.55, -0.79], [0.15, -0.55]])
    }
}


pitch_yaw_roll = {
    "X": "Pitch",
    "Y": "Yaw",
    "Z": "Roll"
}

plt.ion()
fig, axs = plt.subplots(2, 3, figsize=(14, 7))  # 2 rows, 3 cols
fig.subplots_adjust(hspace=0.4, wspace=0.25)



def draw_orientation(ax, angle, axis):
    ax.clear()
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_aspect('equal')
    ax.axis('off')

    rotation = pitch_yaw_roll[axis.split("Orientation")[1]]

    model = plane_models[axis]
    offset = np.array([0, 0])

    if rotation == 'Pitch':
        rotation_matrix = np.array([[np.cos(np.radians(angle-90)), -np.sin(np.radians(angle-90))],
                                    [np.sin(np.radians(angle-90)), np.cos(np.radians(angle-90))]])
    else:
        rotation_matrix = np.array([[np.cos(np.radians(angle)), -np.sin(np.radians(angle))],
                                    [np.sin(np.radians(angle)), np.cos(np.radians(angle))]])

    for part_name, part in model.items():
        rotated_part = np.dot(part * 2, rotation_matrix)
        polygon = patches.Polygon(rotated_part + offset, closed=True, color='blue')
        ax.add_patch(polygon)

    for r in np.linspace(0.4, 2.8, 7):
        circle = plt.Circle(offset, r, fill=False, color='gray', linestyle='--', linewidth=0.5)
        ax.add_patch(circle)

    for _angle in np.linspace(0, 360, 12):
        ax.plot([offset[0], offset[0] + 3*np.sin(np.radians(_angle))],
                [offset[1], offset[1] + 3*np.cos(np.radians(_angle))], 
                'gray', linestyle='--', linewidth=0.5)

    ax.text(-2.8, -2.8, f"{angle}°", ha='left', va='bottom', fontsize=12)
    ax.set_title(f'{rotation}', fontsize=10, pad=-10)


def plot_latest_data(df, window_size=10, ref_pressure=1013.25):
    start = time.time()
    latest_time = df['Timestamp'].max()
    window_start_time = latest_time - window_size
    df_window = df[df['Timestamp'] >= window_start_time]

    # Plot X and Y position in a single plot
    ax_xy = axs[0, 0]
    ax_xy.clear()
    ax_xy.set_xlim(-1000, 1000)
    ax_xy.set_ylim(-1000, 1000)
    ax_xy.plot(df_window['PositionX'], df_window['PositionY'], label='Trajectory', color='gray')
    ax_xy.scatter(df_window['PositionX'].iloc[-1], df_window['PositionY'].iloc[-1], color='red', s=100)  # Highlight last point
    ax_xy.set_xlabel('Position X (m)')
    ax_xy.set_ylabel('Position Y (m)')
    ax_xy.set_title('Position X vs. Y', fontsize=10)
    ax_xy.grid(True)

    # Altitude plot
    ax_altitude = axs[0, 1]
    ax_altitude.clear()
    ax_altitude.set_ylim(-100, 500)
    ax_altitude.set_xlim(window_start_time, latest_time)
    # Convert barometer data to altitude using the barometric formula
    altitude_from_barometer = (1 - (df_window['Barometer'] / ref_pressure)**(1/5.25588)) * 44330.77  # Simplified formula
    ax_altitude.plot(df_window['PositionZ'], label='Altitude from Z', color='b')
    ax_altitude.plot(altitude_from_barometer, label='Altitude from Barometer', color='orange')
    ax_altitude.set_title('Altitude Estimations', fontsize=10)
    ax_altitude.grid(True)
    ax_altitude.legend()

    # Text display moved to the third subplot of the first row
    ax_text = axs[0, 2]
    ax_text.clear()
    ax_text.axis('off')
    distance = np.sqrt(df_window['PositionX'].iloc[-1]**2 + df_window['PositionY'].iloc[-1]**2)
    temperature = df_window['Thermometer'].iloc[-1]
    voltage = df_window['Voltage'].iloc[-1]
    speed = np.sqrt(df_window['VelocityX'].iloc[-1]**2 + df_window['VelocityY'].iloc[-1]**2 + df_window['VelocityZ'].iloc[-1]**2)
    ax_text.text(0.5, 0.6, f"Speed: {speed:.2f} m/s", ha='center', fontsize=12, fontweight='bold')
    ax_text.text(0.5, 0.3, f"Temperature: {temperature:.2f} °C", ha='center', fontsize=12, fontweight='bold')
    ax_text.text(0.5, 0.4, f"Voltage: {voltage:.2f} V", ha='center', fontsize=12, fontweight='bold')
    ax_text.text(0.5, 0.5, f"Distance: {distance:.2f} m", ha='center', fontsize=12, fontweight='bold')

    # Orientation plots remain the same
    for i, axis in enumerate(['OrientationX', 'OrientationY', 'OrientationZ']):
        ax = axs[1, i % 3]
        draw_orientation(ax, df_window[axis].iloc[-1], axis)

    plt.draw()
    plt.pause(0.01)
    fig.canvas.flush_events()

    logging.info(f"Plotting took {time.time() - start:.2f} seconds")


