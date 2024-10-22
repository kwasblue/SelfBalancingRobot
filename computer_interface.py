import dearpygui.dearpygui as dpg
import numpy as np
import time
import paho.mqtt.client as mqtt
import json
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
MQTT_BROKER = "10.243.82.33"
DATA_TOPIC = "ESP32/data"
GAINS_TOPIC = "ESP32/gains"

# Data buffers for angle, gyro, PWM, and corresponding time buffer
data_buffer = {
    'anglex': np.zeros(1000),  # Buffer for anglex
    'gyroX': np.zeros(1000),   # Buffer for GyroX
    'pwm': np.zeros(1000)      # Buffer for PWM
}
time_buffer = np.zeros(1000)  # Buffer to store time data for x-axis

# Gains variables and timers
balancing_gains = {"Kp": 0.0, "Kd": 0.0, "Ki": 0.0}
following_gains = {"Kp2": 0.0, "Kd2": 0.0, "Ki2": 0.0}
last_change_time = time.time()  # Initialize the last change time
gains_pending = False  # Flag to track if a change occurred and needs to be published
start_time = time.time()
# MQTT Setup
client = mqtt.Client()
client.connect(MQTT_BROKER)
client.loop_start()

# ---------------------------
# Helper Functions
# ---------------------------

# Function to handle publishing the new gains if there are no changes for 2 seconds
def publish_gains_if_stable():
    global last_change_time, gains_pending
    while True:
        if gains_pending and (time.time() - last_change_time >= 2):
            # Format the JSON payload with gains rounded to 3 decimal places
            payload = json.dumps({
                "balancing": f"[{balancing_gains['Kp']:.3f},{balancing_gains['Kd']:.3f},{balancing_gains['Ki']:.3f}]",
                "following": f"[{following_gains['Kp2']:.3f},{following_gains['Kd2']:.3f},{following_gains['Ki2']:.3f}]"
            })
            # Publish the gains to the ESP32/gains topic
            client.publish(GAINS_TOPIC, payload)
            print(f"Published Gains: {payload}")
            gains_pending = False  # Reset the pending flag after publishing
        time.sleep(0.1)

# Start a background thread for publishing gains
threading.Thread(target=publish_gains_if_stable, daemon=True).start()

# Callback functions for the sliders and number inputs
def update_kp(sender, app_data):
    global last_change_time, gains_pending
    balancing_gains["Kp"] = app_data
    dpg.set_value("kp_label", f"Kp: {app_data:.3f}")
    dpg.set_value("kp_slider", app_data)  # Update the slider to reflect the number input
    last_change_time = time.time()
    gains_pending = True

def update_kp_slider(sender, app_data):
    update_kp(sender, app_data)  # Use the same update logic for sliders

def update_kd(sender, app_data):
    global last_change_time, gains_pending
    balancing_gains["Kd"] = app_data
    dpg.set_value("kd_label", f"Kd: {app_data:.3f}")
    dpg.set_value("kd_slider", app_data)
    last_change_time = time.time()
    gains_pending = True

def update_kd_slider(sender, app_data):
    update_kd(sender, app_data)

def update_ki(sender, app_data):
    global last_change_time, gains_pending
    balancing_gains["Ki"] = app_data
    dpg.set_value("ki_label", f"Ki: {app_data:.3f}")
    dpg.set_value("ki_slider", app_data)
    last_change_time = time.time()
    gains_pending = True

def update_ki_slider(sender, app_data):
    update_ki(sender, app_data)

def update_kp2(sender, app_data):
    global last_change_time, gains_pending
    following_gains["Kp2"] = app_data
    dpg.set_value("kp2_label", f"Kp2: {app_data:.3f}")
    dpg.set_value("kp2_slider", app_data)
    last_change_time = time.time()
    gains_pending = True

def update_kp2_slider(sender, app_data):
    update_kp2(sender, app_data)

def update_kd2(sender, app_data):
    global last_change_time, gains_pending
    following_gains["Kd2"] = app_data
    dpg.set_value("kd2_label", f"Kd2: {app_data:.3f}")
    dpg.set_value("kd2_slider", app_data)
    last_change_time = time.time()
    gains_pending = True

def update_kd2_slider(sender, app_data):
    update_kd2(sender, app_data)

def update_ki2(sender, app_data):
    global last_change_time, gains_pending
    following_gains["Ki2"] = app_data
    dpg.set_value("ki2_label", f"Ki2: {app_data:.3f}")
    dpg.set_value("ki2_slider", app_data)
    last_change_time = time.time()
    gains_pending = True

def update_ki2_slider(sender, app_data):
    update_ki2(sender, app_data)

# ---------------------------
# GUI Setup
# ---------------------------

dpg.create_context()
dpg.create_viewport(title='Robot Data Viewer', width=WINDOW_WIDTH, height=WINDOW_HEIGHT)

# Create a theme for each plot line (Angle, GyroX, PWM)
with dpg.theme(tag="angle_theme"):
    with dpg.theme_component(dpg.mvLineSeries):
        dpg.add_theme_color(dpg.mvPlotCol_Line, (255, 0, 0), category=dpg.mvThemeCat_Plots)

with dpg.theme(tag="gyro_x_theme"):
    with dpg.theme_component(dpg.mvLineSeries):
        dpg.add_theme_color(dpg.mvPlotCol_Line, (0, 255, 0), category=dpg.mvThemeCat_Plots)

with dpg.theme(tag="pwm_theme"):
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
                    dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag="xaxis_angle")
                    y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Angle")
                    angle_series = dpg.add_line_series(time_buffer, data_buffer['anglex'], label="Angle Data", parent=y_axis)
                    dpg.bind_item_theme(angle_series, "angle_theme")
                
                # Angular Velocity Plot with GyroX
                with dpg.plot(label="Angular Velocity Plot", height=PLOT_HEIGHT, width=PLOT_WIDTH):
                    x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag="xaxis_gyro")
                    y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="GyroX", tag="yaxis_gyro")
                    dpg.set_axis_limits(y_axis, 0, 600)  # Scale y-axis from 0 to 600 rad/s
                    
                    gyro_x_series = dpg.add_line_series(time_buffer, data_buffer['gyroX'], label="GyroX", parent=y_axis)
                    dpg.bind_item_theme(gyro_x_series, "gyro_x_theme")

            # Group for bottom two plots (displacement and PWM)
            with dpg.group():
                # Displacement Plot (Placeholder)
                with dpg.plot(label="Displacement Plot", height=PLOT_HEIGHT, width=PLOT_WIDTH):
                    dpg.add_plot_axis(dpg.mvXAxis, label="X Axis")
                    y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Displacement")
                    dpg.add_line_series(np.linspace(0, 10, 100), np.tan(np.linspace(0, 10, 100)), label="Displacement Data", parent=y_axis)
                
                # PWM Plot
                with dpg.plot(label="PWM Plot", height=PLOT_HEIGHT, width=PLOT_WIDTH):
                    dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag="xaxis_pwm")
                    y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="PWM")
                    pwm_series = dpg.add_line_series(time_buffer, data_buffer['pwm'], label="PWM Data", parent=y_axis)
                    dpg.bind_item_theme(pwm_series, "pwm_theme")

        # Group for sliders and number inputs (Balancing Controller and Following Controller)
        with dpg.group(horizontal=True):
            # Balancing Controller Inputs and Sliders
            with dpg.group():
                dpg.add_text("Kp: 0.00", tag="kp_label")
                dpg.add_input_float(label="Kp Input", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_kp)
                dpg.add_slider_float(label="Kp", tag="kp_slider", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_kp_slider)
                
                dpg.add_text("Kd: 0.00", tag="kd_label")
                dpg.add_input_float(label="Kd Input", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_kd)
                dpg.add_slider_float(label="Kd", tag="kd_slider", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_kd_slider)
                
                dpg.add_text("Ki: 0.00", tag="ki_label")
                dpg.add_input_float(label="Ki Input", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_ki)
                dpg.add_slider_float(label="Ki", tag="ki_slider", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_ki_slider)

            # Spacer between Balancing and Following sliders
            dpg.add_spacer(width=SPACING_BETWEEN_GROUPS)

            # Following Controller Inputs and Sliders
            with dpg.group():
                dpg.add_text("Kp2: 0.00", tag="kp2_label")
                dpg.add_input_float(label="Kp2 Input", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_kp2)
                dpg.add_slider_float(label="Kp2", tag="kp2_slider", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_kp2_slider)
                
                dpg.add_text("Kd2: 0.00", tag="kd2_label")
                dpg.add_input_float(label="Kd2 Input", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_kd2)
                dpg.add_slider_float(label="Kd2", tag="kd2_slider", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_kd2_slider)
                
                dpg.add_text("Ki2: 0.00", tag="ki2_label")
                dpg.add_input_float(label="Ki2 Input", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_ki2)
                dpg.add_slider_float(label="Ki2", tag="ki2_slider", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_ki2_slider)

        # Add the button for "Following Activated"
        dpg.add_button(label="Following", tag="following_button", width=SLIDER_WIDTH)

# ---------------------------
# Main Loop
# ---------------------------

dpg.setup_dearpygui()
dpg.show_viewport()

# Update the plots in a loop
try:
    while dpg.is_dearpygui_running():
        # Use only the last 10 seconds of data (e.g., current_time - 10 to current_time)
        current_time = time.time() - start_time
        mask = time_buffer >= max(0, current_time - 10)

        # Update plots
        dpg.set_value(angle_series, [time_buffer[mask], data_buffer['anglex'][mask]])
        dpg.set_value(gyro_x_series, [time_buffer[mask], data_buffer['gyroX'][mask]])
        dpg.set_value(pwm_series, [time_buffer[mask], data_buffer['pwm'][mask]])

        # Update the x-axis limits for all plots to reflect the moving time window
        dpg.set_axis_limits("xaxis_angle", max(0, current_time - 10), current_time)
        dpg.set_axis_limits("xaxis_gyro", max(0, current_time - 10), current_time)
        dpg.set_axis_limits("xaxis_pwm", max(0, current_time - 10), current_time)

        dpg.render_dearpygui_frame()  # Render the GUI frame
        time.sleep(0.01)  # Add a small delay to limit updates and improve performance
except KeyboardInterrupt:
    print("Window Closing")
    print("---------------------")
# Clean up Dear PyGui context
dpg.destroy_context()

# Disconnect MQTT client
client.disconnect()
