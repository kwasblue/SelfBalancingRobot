import dearpygui.dearpygui as dpg
import numpy as np
import time
import paho.mqtt.client as mqtt
import threading

# ---------------------------
# Configurable Variables
# ---------------------------

WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800
PLOT_WIDTH = 350
PLOT_HEIGHT = 200
SLIDER_WIDTH = 200
SPACING_BETWEEN_GROUPS = 30  # Spacing between sets of sliders

# MQTT Configurations
MQTT_BROKER = "10.0.0.25"
GYRO_TOPIC = "gyro/data"

# Gyroscope data buffer for GyroX, GyroY, GyroZ and corresponding time buffer
gyro_data_buffer = {
    'GyroX': np.zeros(1000),  # Increased size for 10 seconds of data (assuming 100 Hz)
    'GyroY': np.zeros(1000),
    'GyroZ': np.zeros(1000)
}
time_buffer = np.zeros(1000)  # Buffer to store time data for x-axis

# ---------------------------
# MQTT Setup
# ---------------------------

client = mqtt.Client()
client.connect(MQTT_BROKER)
client.loop_start()

# ---------------------------
# Helper Functions
# ---------------------------

start_time = time.time()

# Function to handle incoming gyroscope data and update the buffer
def on_gyro_data(client, userdata, message):
    payload = message.payload.decode('utf-8')
    print(f"Received message: {payload}")  # Print the received MQTT message
    try:
        data = dict(item.split(':') for item in payload.split(','))
        current_time = time.time() - start_time

        # Update the time buffer with the current time (moving window of 10 seconds)
        time_buffer[:] = np.roll(time_buffer, -1)
        time_buffer[-1] = current_time

        # Update gyro buffers with new values
        if 'GyroX' in data:
            gyro_data_buffer['GyroX'] = np.roll(gyro_data_buffer['GyroX'], -1)
            gyro_data_buffer['GyroX'][-1] = float(data['GyroX'])

        if 'GyroY' in data:
            gyro_data_buffer['GyroY'] = np.roll(gyro_data_buffer['GyroY'], -1)
            gyro_data_buffer['GyroY'][-1] = float(data['GyroY'])

        if 'GyroZ' in data:
            gyro_data_buffer['GyroZ'] = np.roll(gyro_data_buffer['GyroZ'], -1)
            gyro_data_buffer['GyroZ'][-1] = float(data['GyroZ'])

    except Exception as e:
        print(f"Error processing message: {e}")

# Subscribe to the 'gyro/data' topic
client.subscribe(GYRO_TOPIC)
client.message_callback_add(GYRO_TOPIC, on_gyro_data)

# ---------------------------
# GUI Setup
# ---------------------------

dpg.create_context()
dpg.create_viewport(title='Robot Data Viewer', width=WINDOW_WIDTH, height=WINDOW_HEIGHT)

# Create a theme for each plot line (GyroX, GyroY, GyroZ)
with dpg.theme(tag="gyro_x_theme"):
    with dpg.theme_component(dpg.mvLineSeries):
        dpg.add_theme_color(dpg.mvPlotCol_Line, (255, 0, 0), category=dpg.mvThemeCat_Plots)

with dpg.theme(tag="gyro_y_theme"):
    with dpg.theme_component(dpg.mvLineSeries):
        dpg.add_theme_color(dpg.mvPlotCol_Line, (0, 255, 0), category=dpg.mvThemeCat_Plots)

with dpg.theme(tag="gyro_z_theme"):
    with dpg.theme_component(dpg.mvLineSeries):
        dpg.add_theme_color(dpg.mvPlotCol_Line, (0, 0, 255), category=dpg.mvThemeCat_Plots)

# Main GUI layout
with dpg.window(label="Robot Data Viewer", width=WINDOW_WIDTH, height=WINDOW_HEIGHT):
    with dpg.group(horizontal=False):  # Group the content vertically
    
        # 2x2 Grid of plots
        with dpg.group(horizontal=True):
            # Group for top two plots (angle and angular velocity)
            with dpg.group():
                # Angle Plot
                with dpg.plot(label="Angle Plot", height=PLOT_HEIGHT, width=PLOT_WIDTH):
                    dpg.add_plot_axis(dpg.mvXAxis, label="X Axis")
                    y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Angle")
                    dpg.add_line_series(np.linspace(0, 10, 100), np.sin(np.linspace(0, 10, 100)), label="Angle Data", parent=y_axis)
                
                # Angular Velocity Plot with GyroX, GyroY, GyroZ
                with dpg.plot(label="Angular Velocity Plot", height=PLOT_HEIGHT, width=PLOT_WIDTH):
                    x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag="xaxis")
                    y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Angular Velocity (rad/s)", tag="yaxis")
                    dpg.set_axis_limits(y_axis, 0, 600)  # Scale y-axis from 0 to 600 rad/s
                    
                    gyro_x_series = dpg.add_line_series(time_buffer, gyro_data_buffer['GyroX'], label="GyroX", parent=y_axis)
                    gyro_y_series = dpg.add_line_series(time_buffer, gyro_data_buffer['GyroY'], label="GyroY", parent=y_axis)
                    gyro_z_series = dpg.add_line_series(time_buffer, gyro_data_buffer['GyroZ'], label="GyroZ", parent=y_axis)

                    # Apply themes to the series
                    dpg.bind_item_theme(gyro_x_series, "gyro_x_theme")
                    dpg.bind_item_theme(gyro_y_series, "gyro_y_theme")
                    dpg.bind_item_theme(gyro_z_series, "gyro_z_theme")

            # Group for bottom two plots (displacement and PWM)
            with dpg.group():
                # Displacement Plot
                with dpg.plot(label="Displacement Plot", height=PLOT_HEIGHT, width=PLOT_WIDTH):
                    dpg.add_plot_axis(dpg.mvXAxis, label="X Axis")
                    y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Displacement")
                    dpg.add_line_series(np.linspace(0, 10, 100), np.tan(np.linspace(0, 10, 100)), label="Displacement Data", parent=y_axis)
                
                # PWM Plot
                with dpg.plot(label="PWM Plot", height=PLOT_HEIGHT, width=PLOT_WIDTH):
                    dpg.add_plot_axis(dpg.mvXAxis, label="X Axis")
                    y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="PWM")
                    dpg.add_line_series(np.linspace(0, 10, 100), np.abs(np.sin(np.linspace(0, 10, 100))), label="PWM Data", parent=y_axis)

        # Group for sliders (Balancing Controller and Following Controller)
        with dpg.group(horizontal=True):
            # Balancing Controller Sliders (stacked vertically)
            with dpg.group():
                dpg.add_text("Kp: 0.00", tag="kp_label")
                dpg.add_slider_float(label="Kp", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH)
                
                dpg.add_text("Kd: 0.00", tag="kd_label")
                dpg.add_slider_float(label="Kd", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH)
                
                dpg.add_text("Ki: 0.00", tag="ki_label")
                dpg.add_slider_float(label="Ki", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH)

            # Spacer between Balancing and Following sliders
            dpg.add_spacer(width=SPACING_BETWEEN_GROUPS)

            # Following Controller Sliders (stacked vertically)
            with dpg.group():
                dpg.add_text("Kp2: 0.00", tag="kp2_label")
                dpg.add_slider_float(label="Kp2", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH)
                
                dpg.add_text("Kd2: 0.00", tag="kd2_label")
                dpg.add_slider_float(label="Kd2", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH)
                
                dpg.add_text("Ki2: 0.00", tag="ki2_label")
                dpg.add_slider_float(label="Ki2", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH)

        # Add the button for "Following Activated"
        dpg.add_button(label="Following", tag="following_button", width=SLIDER_WIDTH)

# ---------------------------
# Main Loop
# ---------------------------

dpg.setup_dearpygui()
dpg.show_viewport()

# Update the gyro plot data in a loop
try:
    while dpg.is_dearpygui_running():
        # Use only the last 10 seconds of data (e.g., current_time - 10 to current_time)
        current_time = time.time() - start_time
        mask = time_buffer >= max(0, current_time - 10)

        print(f"Updating plots: GyroX: {len(gyro_data_buffer['GyroX'][mask])}, "
              f"GyroY: {len(gyro_data_buffer['GyroY'][mask])}, "
              f"GyroZ: {len(gyro_data_buffer['GyroZ'][mask])}")

        # Update all three plots
        dpg.set_value(gyro_x_series, [time_buffer[mask], gyro_data_buffer['GyroX'][mask]])
        dpg.set_value(gyro_y_series, [time_buffer[mask], gyro_data_buffer['GyroY'][mask]])
        dpg.set_value(gyro_z_series, [time_buffer[mask], gyro_data_buffer['GyroZ'][mask]])

        # Update the x-axis to reflect the moving time window
        dpg.set_axis_limits("xaxis", max(0, current_time - 10), current_time)

        dpg.render_dearpygui_frame()  # Render the GUI frame
        time.sleep(0.01)  # Add a small delay to limit updates and improve performance
except KeyboardInterrupt:
    print("Window Closing")
    print("---------------------")
# Clean up Dear PyGui context
dpg.destroy_context()

# Disconnect MQTT client
client.disconnect()