import dearpygui.dearpygui as dpg
import numpy as np
import time

# ---------------------------
# Configurable Variables
# ---------------------------

# Interface dimensions
WINDOW_WIDTH = 700
WINDOW_HEIGHT = 700
PLOT_WIDTH = 400
PLOT_HEIGHT = 200
SLIDER_WIDTH = 200
SPACING_BETWEEN_GROUPS = 10  # Spacing between sets of sliders

# Labels for controllers
BALANCING_CONTROLLER_LABEL = "Balancing Controller"
FOLLOWING_CONTROLLER_LABEL = "Following Controller"
KP_LABEL = "Kp"
KD_LABEL = "Kd"
KI_LABEL = "Ki"
KP2_LABEL = "Kp2"
KD2_LABEL = "Kd2"
KI2_LABEL = "Ki2"

# Button properties
BUTTON_DEFAULT_COLOR = (200, 200, 200, 255)  # Default button color (RGBA)
BUTTON_PRESSED_COLOR = (0, 255, 0, 255)  # Button color when pressed (RGBA)

# ---------------------------
# Dummy data for plots
# ---------------------------

x_data = np.linspace(0, 10, 100)
angle_data = np.sin(x_data)
angular_velocity_data = np.cos(x_data)
displacement_data = np.tan(x_data)

# ---------------------------
# Global Variables
# ---------------------------

# Track changes for both controllers
last_update_time_balancing = time.time()
last_update_time_following = time.time()

# Balancing Controller Gains
current_values = {'kp': 0.0, 'kd': 0.0, 'ki': 0.0}
last_printed_values = {'kp': 0.0, 'kd': 0.0, 'ki': 0.0}
printed_flag_balancing = False

# Following Controller Gains
current_values2 = {'kp2': 0.0, 'kd2': 0.0, 'ki2': 0.0}
last_printed_values2 = {'kp2': 0.0, 'kd2': 0.0, 'ki2': 0.0}
printed_flag_following = False

# Button state to toggle color
button_pressed = False

# ---------------------------
# Helper Functions
# ---------------------------

# Function to print Balancing Controller gains if unchanged for 1 second
def print_balancing_gains_if_stable():
    global printed_flag_balancing
    current_time = time.time()
    
    if current_time - last_update_time_balancing >= 1.0 and not printed_flag_balancing:
        if current_values != last_printed_values:
            print(f"{BALANCING_CONTROLLER_LABEL} Gains: Kp={current_values['kp']:.2f}, Kd={current_values['kd']:.2f}, Ki={current_values['ki']:.2f}")
            last_printed_values.update(current_values)
            printed_flag_balancing = True

# Function to print Following Controller gains if unchanged for 1 second
def print_following_gains_if_stable():
    global printed_flag_following
    current_time = time.time()
    
    if current_time - last_update_time_following >= 1.0 and not printed_flag_following:
        if current_values2 != last_printed_values2:
            print(f"{FOLLOWING_CONTROLLER_LABEL} Gains: Kp2={current_values2['kp2']:.2f}, Kd2={current_values2['kd2']:.2f}, Ki2={current_values2['ki2']:.2f}")
            last_printed_values2.update(current_values2)
            printed_flag_following = True

# Button callback for "Following Activated"
def following_button_callback(sender, app_data):
    global button_pressed
    button_pressed = not button_pressed
    if button_pressed:
        dpg.bind_item_theme("following_button", pressed_button_theme)
        print("Following On")
    else:
        dpg.bind_item_theme("following_button", default_button_theme)
        print("Following Off")

# ---------------------------
# Callbacks
# ---------------------------

# Callback functions for Balancing Controller sliders
def update_kp(sender, app_data):
    global last_update_time_balancing, printed_flag_balancing
    current_values['kp'] = app_data
    last_update_time_balancing = time.time()
    printed_flag_balancing = False
    dpg.set_value("kp_label", f"{KP_LABEL}: {app_data:.2f}")

def update_kd(sender, app_data):
    global last_update_time_balancing, printed_flag_balancing
    current_values['kd'] = app_data
    last_update_time_balancing = time.time()
    printed_flag_balancing = False
    dpg.set_value("kd_label", f"{KD_LABEL}: {app_data:.2f}")

def update_ki(sender, app_data):
    global last_update_time_balancing, printed_flag_balancing
    current_values['ki'] = app_data
    last_update_time_balancing = time.time()
    printed_flag_balancing = False
    dpg.set_value("ki_label", f"{KI_LABEL}: {app_data:.2f}")

# Callback functions for Following Controller sliders
def update_kp2(sender, app_data):
    global last_update_time_following, printed_flag_following
    current_values2['kp2'] = app_data
    last_update_time_following = time.time()
    printed_flag_following = False
    dpg.set_value("kp2_label", f"{KP2_LABEL}: {app_data:.2f}")

def update_kd2(sender, app_data):
    global last_update_time_following, printed_flag_following
    current_values2['kd2'] = app_data
    last_update_time_following = time.time()
    printed_flag_following = False
    dpg.set_value("kd2_label", f"{KD2_LABEL}: {app_data:.2f}")

def update_ki2(sender, app_data):
    global last_update_time_following, printed_flag_following
    current_values2['ki2'] = app_data
    last_update_time_following = time.time()
    printed_flag_following = False
    dpg.set_value("ki2_label", f"{KI2_LABEL}: {app_data:.2f}")

# ---------------------------
# GUI Setup
# ---------------------------

# Create context and viewport
dpg.create_context()
dpg.create_viewport(title='Robot Data Viewer', width=WINDOW_WIDTH, height=WINDOW_HEIGHT)

# Create button themes for color changes
with dpg.theme(tag="default_button_theme") as default_button_theme:
    with dpg.theme_component(dpg.mvButton):
        dpg.add_theme_color(dpg.mvThemeCol_Button, BUTTON_DEFAULT_COLOR)

with dpg.theme(tag="pressed_button_theme") as pressed_button_theme:
    with dpg.theme_component(dpg.mvButton):
        dpg.add_theme_color(dpg.mvThemeCol_Button, BUTTON_PRESSED_COLOR)

# Main GUI layout
with dpg.window(label="Robot Data Viewer", width=WINDOW_WIDTH, height=WINDOW_HEIGHT):
    with dpg.group(horizontal=True):  # Horizontal layout to align plots and sliders

        # Group for the plots
        with dpg.group():
            # Plot for angle
            with dpg.plot(label="Angle Plot", height=PLOT_HEIGHT, width=PLOT_WIDTH):
                dpg.add_plot_axis(dpg.mvXAxis, label="X Axis")
                y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Angle")
                dpg.add_line_series(x_data, angle_data, label="Angle Data", parent=y_axis)

            # Plot for angular velocity
            with dpg.plot(label="Angular Velocity Plot", height=PLOT_HEIGHT, width=PLOT_WIDTH):
                dpg.add_plot_axis(dpg.mvXAxis, label="X Axis")
                y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Angular Velocity")
                dpg.add_line_series(x_data, angular_velocity_data, label="Angular Velocity Data", parent=y_axis)

            # Plot for linear displacement
            with dpg.plot(label="Linear Displacement Plot", height=PLOT_HEIGHT, width=PLOT_WIDTH):
                dpg.add_plot_axis(dpg.mvXAxis, label="X Axis")
                y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Displacement")
                dpg.add_line_series(x_data, displacement_data, label="Displacement Data", parent=y_axis)

        # Group for the sliders (to the right)
        with dpg.group():
            # Balancing Controller Label
            dpg.add_text(BALANCING_CONTROLLER_LABEL)
            # Sliders and labels for Balancing Controller Kp, Kd, Ki
            dpg.add_text(f"{KP_LABEL}: 0.00", tag="kp_label")
            dpg.add_slider_float(label=KP_LABEL, default_value=0, min_value=0, max_value=10, callback=update_kp, width=SLIDER_WIDTH)

            dpg.add_text(f"{KD_LABEL}: 0.00", tag="kd_label")
            dpg.add_slider_float(label=KD_LABEL, default_value=0, min_value=0, max_value=10, callback=update_kd, width=SLIDER_WIDTH)

            dpg.add_text(f"{KI_LABEL}: 0.00", tag="ki_label")
            dpg.add_slider_float(label=KI_LABEL, default_value=0, min_value=0, max_value=10, callback=update_ki, width=SLIDER_WIDTH)

            # Spacer
            dpg.add_spacing(count=SPACING_BETWEEN_GROUPS)

            # Following Controller Label
            dpg.add_text(FOLLOWING_CONTROLLER_LABEL)
            # Sliders and labels for Following Controller Kp, Kd, Ki
            dpg.add_text(f"{KP2_LABEL}: 0.00", tag="kp2_label")
            dpg.add_slider_float(label=KP2_LABEL, default_value=0, min_value=0, max_value=10, callback=update_kp2, width=SLIDER_WIDTH)

            dpg.add_text(f"{KD2_LABEL}: 0.00", tag="kd2_label")
            dpg.add_slider_float(label=KD2_LABEL, default_value=0, min_value=0, max_value=10, callback=update_kd2, width=SLIDER_WIDTH)

            dpg.add_text(f"{KI2_LABEL}: 0.00", tag="ki2_label")
            dpg.add_slider_float(label=KI2_LABEL, default_value=0, min_value=0, max_value=10, callback=update_ki2, width=SLIDER_WIDTH)

            # Normal button added below sliders with color change functionality
            dpg.add_button(label="Following", tag="following_button", callback=following_button_callback, width=SLIDER_WIDTH)
            dpg.bind_item_theme("following_button", default_button_theme)

# ---------------------------
# Main Loop
# ---------------------------

# Setup and show the viewport
dpg.setup_dearpygui()
dpg.show_viewport()

# Continuously check for stable gains to print
try:
    while dpg.is_dearpygui_running():
        print_balancing_gains_if_stable()  # Check if balancing gains need to be printed
        print_following_gains_if_stable()  # Check if following gains need to be printed
        dpg.render_dearpygui_frame()
except KeyboardInterrupt:
    print("Window Closing")
    print("---------------------")
# Clean up Dear PyGui context
dpg.destroy_context()