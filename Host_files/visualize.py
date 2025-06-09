import serial  # For serial port communication
import time    # For delays, e.g., in reconnection attempts
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
from matplotlib.patches import Circle  # For drawing circles
import math                           # For distance calculation (sqrt)

# Serial port configuration
SERIAL_PORT = 'COM6'  # Specify your COM port
BAUD_RATE = 115200    # Specify the baud rate (must match your device)

# Data buffers for plotting the latest point (using deques with maxlen=1)
MAX_DATA_POINTS = 1  # Store only the most recent data point
unfiltered_plot_x = deque(maxlen=MAX_DATA_POINTS)
unfiltered_plot_y = deque(maxlen=MAX_DATA_POINTS)
unfiltered_plot_x_lws = deque(maxlen=MAX_DATA_POINTS)
unfiltered_plot_y_lws = deque(maxlen=MAX_DATA_POINTS)
filtered_plot_x = deque(maxlen=MAX_DATA_POINTS)
filtered_plot_y = deque(maxlen=MAX_DATA_POINTS)
filtered_plot_x_lws = deque(maxlen=MAX_DATA_POINTS)
filtered_plot_y_lws = deque(maxlen=MAX_DATA_POINTS)

# Thread synchronization
data_lock = threading.Lock()
new_data_available = False

# Set up the figure for a single 2D plot
fig, ax = plt.subplots(figsize=(8, 8))  # Square figure
fig.suptitle('2D Position Visualization', fontsize=16)

# Initialize line objects for the four points
unfiltered_line, = ax.plot([], [], 'ro', label='Unfiltered No LWS', markersize=8)      # Red circle
unfiltered_lws_line, = ax.plot([], [], 'go', label='Unfiltered LWS', markersize=8)     # Green circle
filtered_line, = ax.plot([], [], 'bo', label='Filtered No LWS', markersize=8)          # Blue circle
filtered_lws_line, = ax.plot([], [], 'yo', label='Filtered LWS', markersize=8)         # Yellow circle

ax.set_xlim(0, 5.0)
ax.set_ylim(0, 5.0)
ax.set_xlabel('X coordinate (meters)')
ax.set_ylabel('Y coordinate (meters)')
ax.set_title('Real-time 2D Position Tracking')
ax.grid(True)
ax.set_aspect('equal', adjustable='box')  # Ensure X and Y scales are the same

# --- Add static points for RPis ---
rpi_points_coords = {
    # Store as: "Name": ((x, y), "color")
    "RPi_1": ((0, 0), "magenta"),
    "RPi_2": ((4.5, 0), "cyan"),
    "RPi_3": ((2.25, 3.897), "orange")  # Orange is often more visible than yellow
}
# Common style for RPi markers, color will be overridden
rpi_marker_style = {'marker': 's', 'markersize': 10, 'linestyle': 'None'}

for name, (coords, color) in rpi_points_coords.items():
    px, py = coords
    ax.plot(px, py, label=name, color=color, **rpi_marker_style)
    ax.text(px + 0.05, py + 0.05, name, fontsize=9)  # Offset text slightly

# --- Initialize Circle patches for distances from RPis to filtered_lws point ---
rpi_circles = []
circle_plot_styles = [
    {'color': 'magenta', 'linestyle': '-', 'fill': False, 'linewidth': 2.0},
    {'color': 'cyan', 'linestyle': '-', 'fill': False, 'linewidth': 2.0},
    {'color': 'orange', 'linestyle': '-', 'fill': False, 'linewidth': 2.0}
]

rpi_names_ordered = list(rpi_points_coords.keys())

for i, rpi_name in enumerate(rpi_names_ordered):
    rpi_data = rpi_points_coords[rpi_name]
    px, py = rpi_data[0]  # RPi coordinates are the center of the circle
    circle = Circle((px, py), 0.01, label=f'Dist from {rpi_name}', **circle_plot_styles[i % len(circle_plot_styles)])
    ax.add_patch(circle)
    rpi_circles.append(circle)

ax.legend(fontsize='small')  # Include all labeled artists (points and circles)

def receive_data():
    """Function that receives data from the serial COM port"""
    global new_data_available

    while True:  # Main loop to attempt connection and reading
        ser = None  # Initialize ser to None for the finally block
        try:
            print(f"Attempting to connect to serial port {SERIAL_PORT} at {BAUD_RATE} bps...")
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)  # timeout for read operations
            print(f"Successfully connected to {SERIAL_PORT}.")

            buffer = ""
            while True:  # Reading loop for the currently open serial port
                if ser.in_waiting > 0:
                    data_chunk = ser.read(ser.in_waiting).decode('utf-8', errors='replace')
                    buffer += data_chunk

                    while '\n' in buffer:
                        line_end = buffer.find('\n')
                        json_line = buffer[:line_end].strip()
                        buffer = buffer[line_end + 1:]  # Keep rest for next iteration
                        print(json_line)
                        if not json_line:
                            continue

                        try:
                            sensor_data = json.loads(json_line)

                            if ('unfiltered' in sensor_data and isinstance(sensor_data['unfiltered'], list) and
                                'unfiltered_lws' in sensor_data and isinstance(sensor_data['unfiltered_lws'], list) and
                                'filtered' in sensor_data and isinstance(sensor_data['filtered'], list) and
                                'filtered_lws' in sensor_data and isinstance(sensor_data['filtered_lws'], list)):

                                with data_lock:
                                    for point in sensor_data['unfiltered']:
                                        if isinstance(point, dict) and 'x' in point and 'y' in point:
                                            try:
                                                unfiltered_plot_x.append(float(point['x']))
                                                unfiltered_plot_y.append(float(point['y']))
                                            except ValueError:
                                                print(f"Invalid non-numeric coordinate in 'unfiltered': {point}")
                                        else:
                                            print(f"Malformed point in 'unfiltered': {point}")

                                    for point in sensor_data['unfiltered_lws']:
                                        if isinstance(point, dict) and 'x' in point and 'y' in point:
                                            try:
                                                unfiltered_plot_x_lws.append(float(point['x']))
                                                unfiltered_plot_y_lws.append(float(point['y']))
                                            except ValueError:
                                                print(f"Invalid non-numeric coordinate in 'unfiltered_lws': {point}")
                                        else:
                                            print(f"Malformed point in 'unfiltered_lws': {point}")

                                    for point in sensor_data['filtered']:
                                        if isinstance(point, dict) and 'x' in point and 'y' in point:
                                            try:
                                                filtered_plot_x.append(float(point['x']))
                                                filtered_plot_y.append(float(point['y']))
                                            except ValueError:
                                                print(f"Invalid non-numeric coordinate in 'filtered': {point}")
                                        else:
                                            print(f"Malformed point in 'filtered': {point}")

                                    for point in sensor_data['filtered_lws']:
                                        if isinstance(point, dict) and 'x' in point and 'y' in point:
                                            try:
                                                filtered_plot_x_lws.append(float(point['x']))
                                                filtered_plot_y_lws.append(float(point['y']))
                                            except ValueError:
                                                print(f"Invalid non-numeric coordinate in 'filtered_lws': {point}")
                                        else:
                                            print(f"Malformed point in 'filtered_lws': {point}")
                                    new_data_available = True

                            else:
                                print(f"Invalid data format, missing keys in JSON: {json_line}")

                        except json.JSONDecodeError:
                            pass
                            # print(f"Failed to parse JSON: '{json_line}'")
                        except Exception as e:
                            pass
                            # print(f"Error processing data: {e} from JSON: {json_line}")
                else:
                    time.sleep(0.01)

                if not ser.is_open:
                    print(f"Serial port {SERIAL_PORT} is no longer open. Attempting to reconnect.")
                    break

        except serial.SerialException as e:
            print(f"Serial port error on {SERIAL_PORT}: {e}.")
        except Exception as e:
            print(f"An unexpected error occurred in receive_data: {e}.")
        finally:
            if ser and ser.is_open:
                ser.close()
                print(f"Serial port {SERIAL_PORT} closed.")
            print("Retrying connection in 5 seconds...")
            time.sleep(5)

def update_plot(frame):
    global new_data_available
    artists_to_update = [
        unfiltered_line, unfiltered_lws_line,
        filtered_line, filtered_lws_line
    ]

    if new_data_available:
        with data_lock:
            # unfiltered_no_lws
            if unfiltered_plot_x and unfiltered_plot_y:
                unfiltered_line.set_data([unfiltered_plot_x[0]], [unfiltered_plot_y[0]])
            else:
                unfiltered_line.set_data([], [])

            # unfiltered_lws
            if unfiltered_plot_x_lws and unfiltered_plot_y_lws:
                unfiltered_lws_line.set_data([unfiltered_plot_x_lws[0]], [unfiltered_plot_y_lws[0]])
            else:
                unfiltered_lws_line.set_data([], [])

            # filtered_no_lws
            if filtered_plot_x and filtered_plot_y:
                filtered_line.set_data([filtered_plot_x[0]], [filtered_plot_y[0]])
            else:
                filtered_line.set_data([], [])

            # filtered_lws (circles are based on this)
            if filtered_plot_x_lws and filtered_plot_y_lws:
                current_x = filtered_plot_x_lws[0]
                current_y = filtered_plot_y_lws[0]
                filtered_lws_line.set_data([current_x], [current_y])

                for i, rpi_name in enumerate(rpi_names_ordered):
                    rpi_x, rpi_y = rpi_points_coords[rpi_name][0]
                    distance = math.sqrt((current_x - rpi_x)**2 + (current_y - rpi_y)**2)
                    if i < len(rpi_circles):
                        rpi_circles[i].set_radius(distance)
            else:
                filtered_lws_line.set_data([], [])
                for circle_patch in rpi_circles:
                    circle_patch.set_radius(0.001)

        new_data_available = False

    return tuple(artists_to_update) + tuple(rpi_circles)

# Start the data receiving thread
data_receive_thread = threading.Thread(target=receive_data, daemon=True)
data_receive_thread.start()

# Create animation
ani = animation.FuncAnimation(
    fig, update_plot, interval=100, blit=True, cache_frame_data=False
)

plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust layout for suptitle
plt.show()