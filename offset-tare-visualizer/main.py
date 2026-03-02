"""
Offset/Tare Visualizer for YostLabs Sensors

Dual-view application showing:
- Left: Untared orientation with offset (camera shows tare position)
- Right: Tared orientation (from camera's POV)

See DESIGN.md for full specifications.
"""

import dearpygui.dearpygui as dpg
import math

# Graphics library
from yostlabs.graphics import GL_Context, ModelObject, TransformNode, GL_AXIS_ORDER
from yostlabs.graphics.resources import get_model_path
from yostlabs.graphics.prefabs import TriAxesObject
from yostlabs.graphics.scene_prefabs import OrientationScene
from yostlabs.graphics.dpg import DpgScene, DpgCameraMover

# Sensor API
from yostlabs.tss3.api import ThreespaceSensor, StreamableCommands
from yostlabs.math.axes import AxisOrder
from virtual_sensor import SensorBridge
from yostlabs.math.quaternion import (
    quat_mul, 
    quat_inverse,  
    quat_from_euler,
    quat_to_euler_angles,
    quat_rotate_vec
)

# ==============================================================================
# CONFIGURATION
# ==============================================================================
TEXTURE_WIDTH = 600
TEXTURE_HEIGHT = 600

# ==============================================================================
# GLOBAL STATE
# ==============================================================================
class AppState:
    """Centralized application state (UI and display only)"""
    def __init__(self):
        # Axis order configuration
        self.axis_permutation = "xyz"  # Current permutation
        self.negate_x = False
        self.negate_y = False
        self.negate_z = False
        self.axis_order: AxisOrder = None
        
        # Model selection
        self.current_model = "Embedded"  # Current model type
        
        # Display options (placeholders for future implementation)
        self.show_offset_on_model = False  # If true, rotate model; if false, rotate only axes
        self.show_ghost_base_offset = False
        self.show_ghost_base_tare = False
        self.show_tare_camera = False  # Show tare camera indicator
        
        # Euler angle offset configuration
        self.euler_order = "XYZ"  # Current Euler order
        self.euler_type = "Offset"  # Which quaternion to edit: Offset, Tare, Base Offset, Base Tare, Untared
        self.euler_x = 0.0
        self.euler_y = 0.0
        self.euler_z = 0.0
        self.euler_extrinsic = False  # Intrinsic by default
        
        # Untared orientation source
        self.untared_source = "Sensor"  # "Sensor" or "Euler"
        self.has_real_sensor = False  # Whether a physical sensor is connected
        
        # Rendering control
        self.skip_next_packet = False  # Skip next packet after axis order change
        self._skip_euler_update = False  # Skip Euler editor update during apply_euler_to_sensor

state = AppState()

# ==============================================================================
# SENSOR SETUP
# ==============================================================================
print("Attempting to discover sensor...")
real_sensor = None
try:
    real_sensor = ThreespaceSensor(verbose=True)
    print("Sensor connected successfully!")
    state.has_real_sensor = True
    state.untared_source = "Sensor"  # Use sensor data by default
except Exception as e:
    print(f"No sensor found: {e}")
    print("Running in virtual mode (Euler only)")
    state.has_real_sensor = False
    state.untared_source = "Euler"  # Must use Euler mode without sensor

sensor = SensorBridge(real_sensor=real_sensor, use_real_data=state.has_real_sensor)

# Initialize sensor settings
sensor.set_settings(
    tare_quat=[0, 0, 0, 1],
    offset=[0, 0, 0, 1],
    base_offset=[0, 0, 0, 1],
    base_tare=[0, 0, 0, 1],
    tare_auto_base=True,
    axis_offset_enabled=True
)

# Set up streaming if real sensor exists
sensor.set_settings(
    stream_hz=33,
    stream_slots=[
        StreamableCommands.GetUntaredOrientation,
        StreamableCommands.GetPrimaryCorrectedAccelVec,
        StreamableCommands.GetPrimaryCorrectedGyroRate,
        StreamableCommands.GetPrimaryCorrectedMagVec
    ]
)

# ==============================================================================
# GRAPHICS SETUP
# ==============================================================================
GL_Context.init()

# Model configurations: model_type -> filename
MODEL_CONFIGS = {
    "DL3": "DL-3.obj",
    "Embedded": "EM-3.obj",
}

# Create sensor model (shared geometry, different instances)
def create_sensor_model(name, model_type="DL3"):
    """Create a sensor model instance"""
    filename = MODEL_CONFIGS[model_type]
    model = ModelObject(name, get_model_path(filename))
    return model

# Create camera indicator model
def create_camera_model(name):
    """Create a camera indicator model"""
    camera = ModelObject(name, get_model_path('Camera.obj'))
    camera.set_scale(4/23)
    # Initial position along Z axis (will be rotated based on tare)
    camera.set_position([0, 0, 50/23])
    camera.set_rotation_quat([0, 0, 0, 1])  # Identity rotation - pointing at sensor
    
    # Add axes to camera
    camera_axes = TriAxesObject(f"{name}_Axes")
    camera_axes.set_scale(1/camera.scale[0])
    camera.add_child(camera_axes)
    
    # Store the base offset from origin (camera orbits around this point)
    camera.base_offset = [0, 0, 50/23]
    
    return camera

# OFFSET VIEW: Untared orientation with offset applied (camera shows tare position)
offset_model = create_sensor_model("OffsetSensorModel", state.current_model)

offset_scene = OrientationScene(
    TEXTURE_WIDTH, 
    TEXTURE_HEIGHT, 
    offset_model, 
    font=GL_Context.default_font,
    name="Offset View"
)
offset_scene.set_background_color(0.35, 0.35, 0.4, 1.0)  # Slightly blue-gray
offset_scene.set_camera_mover(DpgCameraMover(offset_scene.camera))

# Add an additional game object to the hierarchy to allow for separation of untared orientation and untared + offset
# for working with additional mathematical parenting models (such as base tare) via parenting. This is specifically to avoid
# issues of using different math for base tare when choosing to show offset only as an axis change or as a model rotation.
# This is done after the scene is created to allow the scene to still create its own axes as children of the main model node.
# This also allows for some additional transform separations like separation of scale.
untared_orientation_gameobject = TransformNode("UntaredOrientationNode")
offset_scene.remove_child(offset_model)
untared_orientation_gameobject.add_child(offset_model)
offset_scene.add_child(untared_orientation_gameobject)

# Axes remain as children of the model (default behavior from OrientationScene)
# They will be rotated by offset when show_offset_on_model is False

# Create ghost axes for base offset visualization
ghost_base_offset_axes = TriAxesObject("GhostBaseOffsetAxes")
ghost_base_offset_axes.set_position([0, 0, 0])
ghost_base_offset_axes.set_alpha(0.3)  # Make it semi-transparent/ghostly
offset_scene.add_child(ghost_base_offset_axes)
ghost_base_offset_axes.set_visible(state.show_ghost_base_offset)  # Initially hidden

# Create ghost camera for base tare visualization (child of sensor model)
ghost_base_tare_camera = create_camera_model("GhostBaseTareCamera")
ghost_base_tare_camera.set_alpha(0.3)  # Make it semi-transparent/ghostly
# Also make the camera's axes transparent
for child in ghost_base_tare_camera.children:
    if isinstance(child, TriAxesObject):
        child.set_alpha(0.3)
untared_orientation_gameobject.add_child(ghost_base_tare_camera)  # Child of model, not scene
ghost_base_tare_camera.set_active(state.show_ghost_base_tare)  # Initially hidden (use active to hide axes too)

# Add camera indicator to offset scene
offset_camera = create_camera_model("OffsetCamera")
offset_scene.add_child(offset_camera)
offset_camera.set_active(state.show_tare_camera)

# Wrap in DpgScene
offset_dpg_scene = DpgScene(TEXTURE_WIDTH, TEXTURE_HEIGHT, scene=offset_scene)

# TARED VIEW: Tared orientation (from camera's POV)
tared_model = create_sensor_model("TaredSensorModel", state.current_model)
tared_scene = OrientationScene(
    TEXTURE_WIDTH, 
    TEXTURE_HEIGHT, 
    tared_model, 
    font=GL_Context.default_font,
    name="Tared View"
)
tared_scene.set_background_color(0.35, 0.4, 0.35, 1.0)  # Slightly green-gray
tared_scene.set_camera_mover(DpgCameraMover(tared_scene.camera))

# Wrap in DpgScene
tared_dpg_scene = DpgScene(TEXTURE_WIDTH, TEXTURE_HEIGHT, scene=tared_scene)

# ==============================================================================
# SENSOR CONTROL CALLBACKS
# ==============================================================================

def set_offset():
    """Set offset with current orientation"""
    sensor.setOffsetWithCurrentOrientation()
    print(f"Offset set: {sensor.offset_quat}")
    if sensor.tare_auto_base:
        print(f"Base tare set via tare_auto_base: {sensor.base_tare_quat}")
    # Trigger Euler editor update if needed
    if state.euler_type == "Offset":
        load_euler_from_quat()
    elif state.euler_type == "Base Tare" and sensor.tare_auto_base:
        load_euler_from_quat()

def reset_offset():
    """Reset offset to identity"""
    sensor.set_settings(offset=[0, 0, 0, 1])
    print("Offset reset")
    if state.euler_type == "Offset":
        load_euler_from_quat()

def set_base_offset():
    """Set base offset with current orientation"""
    sensor.setBaseOffsetWithCurrentOrientation()
    print(f"Base offset set: {sensor.base_offset_quat}")
    if state.euler_type == "Base Offset":
        load_euler_from_quat()

def reset_base_offset():
    """Reset base offset to identity"""
    sensor.set_settings(base_offset=[0, 0, 0, 1])
    print("Base offset reset")
    if state.euler_type == "Base Offset":
        load_euler_from_quat()

def set_tare():
    """Set tare with current orientation"""
    sensor.tareWithCurrentOrientation()
    print(f"Tare set: {sensor.tare_quat}")
    if state.euler_type == "Tare":
        load_euler_from_quat()

def reset_tare():
    """Reset tare to identity"""
    sensor.set_settings(tare_quat=[0, 0, 0, 1])
    print("Tare reset")
    if state.euler_type == "Tare":
        load_euler_from_quat()

def set_base_tare():
    """Set base tare with current orientation"""
    sensor.setBaseTareWithCurrentOrientation()
    print(f"Base tare set: {sensor.base_tare_quat}")
    if state.euler_type == "Base Tare":
        load_euler_from_quat()

def reset_base_tare():
    """Reset base tare to identity"""
    sensor.set_settings(base_tare=[0, 0, 0, 1])
    print("Base tare reset")
    if state.euler_type == "Base Tare":
        load_euler_from_quat()

def toggle_offset_on_model(sender, app_data):
    """Toggle whether offset rotates the entire model or just the axes"""
    state.show_offset_on_model = app_data
    mode = "entire model" if app_data else "axes only"
    print(f"Offset visualization mode: {mode}")

def toggle_ghost_base_offset(sender, app_data):
    """Toggle visibility of ghost base offset axes"""
    state.show_ghost_base_offset = app_data
    ghost_base_offset_axes.set_visible(app_data)
    print(f"{ghost_base_offset_axes.visible=}")
    print(f"Ghost base offset axes: {'visible' if app_data else 'hidden'}")

def toggle_ghost_base_tare(sender, app_data):
    """Toggle visibility of ghost base tare camera"""
    state.show_ghost_base_tare = app_data
    ghost_base_tare_camera.set_active(app_data)
    print(f"Ghost base tare camera: {'active' if app_data else 'inactive'}")

def toggle_tare_camera(sender, app_data):
    """Toggle visibility of tare camera indicator"""
    state.show_tare_camera = app_data
    offset_camera.set_active(app_data)
    print(f"Tare camera: {'active' if app_data else 'inactive'}")

def toggle_tare_auto_base(sender, app_data):
    """Toggle automatic base tare on offset setting"""
    sensor.set_settings(tare_auto_base=app_data)
    print(f"Auto base tare on offset: {'enabled' if app_data else 'disabled'}")

def toggle_axis_offset_enabled(sender, app_data):
    """Toggle axis offset for component data"""
    sensor.set_settings(axis_offset_enabled=app_data)
    print(f"Axis offset enabled for component data: {'enabled' if app_data else 'disabled'}")

def reset_cameras():
    """Reset camera positions and rotations to defaults for both scenes"""
    offset_scene.reset_camera()
    tared_scene.reset_camera()
    print("Cameras reset to default positions")

def load_euler_from_quat():
    """Load quaternion based on euler_type and convert to Euler angles"""
    # Skip update if called from apply_euler_to_sensor to prevent circular updates
    if state._skip_euler_update:
        return
    
    # Get the quaternion based on selected type
    quat_map = {
        "Offset": sensor.offset_quat,
        "Tare": sensor.tare_quat,
        "Base Offset": sensor.base_offset_quat,
        "Base Tare": sensor.base_tare_quat,
        "Untared": sensor.untared_orientation
    }
    quat = quat_map.get(state.euler_type, [0, 0, 0, 1])
    if state.euler_type == "Untared":
        print("Loading untared quat:", quat)
    # Convert quaternion to Euler angles
    euler_rad = quat_to_euler_angles(quat, state.euler_order, extrinsic=state.euler_extrinsic)
    
    # Convert to degrees
    state.euler_x = math.degrees(euler_rad[0])
    state.euler_y = math.degrees(euler_rad[1])
    state.euler_z = math.degrees(euler_rad[2])
    
    # Update UI
    dpg.set_value("euler_angle_0", state.euler_x)
    dpg.set_value("euler_angle_1", state.euler_y)
    dpg.set_value("euler_angle_2", state.euler_z)
    
    # Update labels based on euler order
    order = state.euler_order
    dpg.set_item_label("euler_angle_0", order[0])
    dpg.set_item_label("euler_angle_1", order[1])
    dpg.set_item_label("euler_angle_2", order[2])
    
    print(f"Loaded {state.euler_type} as Euler ({state.euler_order}): {state.euler_x:.2f}, {state.euler_y:.2f}, {state.euler_z:.2f}")

def apply_euler_to_sensor():
    """Convert current Euler angles to quaternion and apply to sensor"""
    # Convert degrees to radians
    euler_rad = [math.radians(state.euler_x), math.radians(state.euler_y), math.radians(state.euler_z)]
    
    # Convert Euler to quaternion
    quat = quat_from_euler(euler_rad, state.euler_order, extrinsic=state.euler_extrinsic)
    
    # Apply based on type
    setting_map = {
        "Offset": "offset",
        "Tare": "tare_quat",
        "Base Offset": "base_offset",
        "Base Tare": "base_tare",
        "Untared": None  # Untared is not a sensor setting, only local state
    }
    setting_name = setting_map.get(state.euler_type)
    
    # Update sensor - skip Euler editor auto-update to prevent circular updates
    state._skip_euler_update = True
    try:
        # Update sensor settings (if applicable)
        if setting_name:
            sensor.set_settings(**{setting_name: quat})
        elif state.euler_type == "Untared":
            sensor.untared_orientation = quat
            # When setting untared via Euler, switch source to Euler
            if state.untared_source == "Sensor":
                state.untared_source = "Euler"
                sensor.use_real_data = False
                dpg.set_value("untared_source_combo", "Euler")
                print("Switched to Euler source for untared orientation")
    finally:
        state._skip_euler_update = False
        
        rotation_type = "extrinsic" if state.euler_extrinsic else "intrinsic"
        print(f"{state.euler_type} set from Euler ({state.euler_order}, {rotation_type}): {state.euler_x:.2f}, {state.euler_y:.2f}, {state.euler_z:.2f}")

def on_euler_type_change(sender, app_data):
    """Callback for Euler type dropdown - loads corresponding quat"""
    state.euler_type = app_data
    load_euler_from_quat()

def on_euler_order_change(sender, app_data):
    """Callback for Euler order dropdown - reloads with new order"""
    state.euler_order = app_data
    load_euler_from_quat()

def on_euler_angle_0_change(sender, app_data):
    """Callback for first Euler angle"""
    state.euler_x = app_data
    apply_euler_to_sensor()

def on_euler_angle_1_change(sender, app_data):
    """Callback for second Euler angle"""
    state.euler_y = app_data
    apply_euler_to_sensor()

def on_euler_angle_2_change(sender, app_data):
    """Callback for third Euler angle"""
    state.euler_z = app_data
    apply_euler_to_sensor()

def on_euler_extrinsic_change(sender, app_data):
    """Callback for extrinsic checkbox - reloads with new setting"""
    state.euler_extrinsic = app_data
    load_euler_from_quat()
    rotation_type = "extrinsic" if app_data else "intrinsic"
    print(f"Euler rotation type: {rotation_type}")

def on_untared_source_change(sender, app_data):
    """Callback for untared source dropdown - switches between sensor and Euler"""
    state.untared_source = app_data
    if app_data == "Sensor":
        sensor.use_real_data = True
        print("Untared source: Sensor (live data)")
    else:  # Euler
        sensor.use_real_data = False
        print(f"Untared source: Euler (frozen at current orientation)")
        # If switching to Euler and type is Untared, load current orientation
        if state.euler_type == "Untared":
            load_euler_from_quat()

def update_axis_order():
    """Update axis order based on current permutation and negation states"""
    # Build axis order string from permutation and negations
    # Negations are based on POSITION (0=X, 1=Y, 2=Z), not the letter at that position
    negations = [state.negate_x, state.negate_y, state.negate_z]
    axis_str = ""
    for i, char in enumerate(state.axis_permutation):
        if negations[i]:  # Negate based on position, not letter
            axis_str += '-' + char
        else:
            axis_str += char
    
    state.axis_order = AxisOrder(axis_str)
    
    # Update sensor
    sensor.set_settings(axis_order=axis_str)
    if state.euler_type == "Untared":
        load_euler_from_quat()
    
    # Update orientation scenes
    offset_scene.set_axis_order(state.axis_order)
    tared_scene.set_axis_order(state.axis_order)
    
    # Update ghost base offset axes
    ghost_base_offset_axes.set_axis_order(state.axis_order)
    
    # Update camera axes
    for camera in [offset_camera, ghost_base_tare_camera]:
        for child in camera.children:
            if isinstance(child, TriAxesObject):
                child.set_axis_order(state.axis_order)
    
    # Get axis_order_c from sensor (compass notation)
    axis_order_c = sensor.get_settings("axis_order_c")
    
    # Update display text - combined format
    dpg.set_value("axis_order_text", f"axis_order: {sensor.axis_order} / {axis_order_c}")
    
    # Skip next packet to avoid rendering old orientation data in new axis order
    state.skip_next_packet = True
    
    print(f"Axis order updated: {sensor.axis_order} (axis_order_c: {axis_order_c})")

def on_axis_permutation_change(sender, app_data):
    """Callback for axis permutation dropdown"""
    state.axis_permutation = app_data
    update_axis_order()

def on_negate_x_change(sender, app_data):
    """Callback for X negation checkbox"""
    state.negate_x = app_data
    update_axis_order()

def on_negate_y_change(sender, app_data):
    """Callback for Y negation checkbox"""
    state.negate_y = app_data
    update_axis_order()

def on_negate_z_change(sender, app_data):
    """Callback for Z negation checkbox"""
    state.negate_z = app_data
    update_axis_order()

def on_model_change(sender, app_data):
    """Callback for model selection dropdown"""
    global offset_model, tared_model, untared_orientation_gameobject
    
    state.current_model = app_data

    #Swap tared model
    tared_rotation = tared_model.rotation
    tared_model = create_sensor_model("TaredSensorModel", state.current_model)
    tared_scene.set_model(tared_model)
    tared_model.set_rotation_matrix(tared_rotation)

    #Swapping offset model is a bit more complicated because the added untared_orientation_gameobject parent
    offset_rotation = offset_model.rotation
    untared_orientation_gameobject.remove_child(offset_model)
    offset_model = create_sensor_model("OffsetSensorModel", state.current_model)
    offset_scene.set_model(offset_model)
    offset_scene.remove_child(offset_model)
    untared_orientation_gameobject.add_child(offset_model)
    
    print(f"Model changed to: {state.current_model}")

# ==============================================================================
# UI SETUP
# ==============================================================================
dpg.create_context()

# Calculate window size for dual display
window_width = (TEXTURE_WIDTH * 2) + 60 + 8
window_height = TEXTURE_HEIGHT + 290  # Increased to prevent scrollbar (includes menubar)

dpg.create_viewport(
    title="Offset/Tare Visualizer",
    width=window_width, 
    height=window_height
)

with dpg.window(label="Main Window", tag="primary_window"):
    
    # Menu bar
    with dpg.menu_bar():
        with dpg.menu(label="Model"):
            for model_name in MODEL_CONFIGS.keys():
                dpg.add_menu_item(
                    label=model_name,
                    callback=lambda s, a, u: on_model_change(s, u),
                    user_data=model_name
                )
        with dpg.menu(label="View"):
            dpg.add_menu_item(
                label="Reset Cameras",
                callback=lambda: reset_cameras()
            )
        with dpg.menu(label="Source"):
            dpg.add_text("Untared Orientation:")
            dpg.add_combo(
                items=["Sensor", "Euler"],
                default_value=state.untared_source,
                callback=on_untared_source_change,
                tag="untared_source_combo",
                width=100,
                enabled=state.has_real_sensor  # Only enabled if real sensor connected
            )
    
    # Dual viewer display
    with dpg.group(horizontal=True):
        with dpg.group():
            dpg.add_text("Offset View (Camera Represents Tare)", color=(200, 200, 255))
            dpg.add_image(
                offset_dpg_scene.createDpgTexture(), 
                width=TEXTURE_WIDTH, 
                height=TEXTURE_HEIGHT
            )
        
        dpg.add_spacer(width=20)
        
        with dpg.group():
            dpg.add_text("Tared View (From Camera POV)", color=(200, 255, 200))
            dpg.add_image(
                tared_dpg_scene.createDpgTexture(), 
                width=TEXTURE_WIDTH, 
                height=TEXTURE_HEIGHT
            )
    
    dpg.add_separator()
    
    # Control buttons - using table for proper column separation
    with dpg.table(header_row=False, borders_innerV=True, policy=dpg.mvTable_SizingFixedFit, pad_outerX=True):
        dpg.add_table_column(init_width_or_weight=180)
        dpg.add_table_column(init_width_or_weight=180)
        dpg.add_table_column(init_width_or_weight=220)
        dpg.add_table_column(init_width_or_weight=160)
        dpg.add_table_column(init_width_or_weight=200)  # Euler angle section
        dpg.add_table_column(init_width_or_weight=340)
        
        with dpg.table_row():
            # Offset controls
            with dpg.table_cell():
                dpg.add_text("OFFSET CONTROLS", color=(255, 200, 100))
                dpg.add_button(label="Set Offset", callback=set_offset, width=150)
                dpg.add_button(label="Reset Offset", callback=reset_offset, width=150)
                dpg.add_separator()
                dpg.add_button(label="Set Base Offset", callback=set_base_offset, width=150)
                dpg.add_button(label="Reset Base Offset", callback=reset_base_offset, width=150)
            
            # Tare controls
            with dpg.table_cell():
                dpg.add_text("TARE CONTROLS", color=(100, 200, 255))
                dpg.add_button(label="Set Tare", callback=set_tare, width=150)
                dpg.add_button(label="Reset Tare", callback=reset_tare, width=150)
                dpg.add_separator()
                dpg.add_button(label="Set Base Tare", callback=set_base_tare, width=150)
                dpg.add_button(label="Reset Base Tare", callback=reset_base_tare, width=150)
            
            # Display options
            with dpg.table_cell():
                dpg.add_text("DISPLAY OPTIONS", color=(200, 255, 100))
                
                # Offset Rotates Model
                with dpg.group(horizontal=True):
                    dpg.add_checkbox(
                        label="Offset Rotates Model", 
                        default_value=state.show_offset_on_model,
                        callback=toggle_offset_on_model
                    )
                    help_text = dpg.add_text("(?)", color=(200, 200, 100))
                    with dpg.tooltip(help_text):
                        dpg.add_text(
                            "Offsetting effectively changes the axes of the sensor. Checking this \n"
                            "box will display the whole model rotated by the offset rather than \n"
                            "just the axes.",
                            wrap=300
                        )
                
                # Show Ghost Base Offset
                with dpg.group(horizontal=True):
                    dpg.add_checkbox(
                        label="Show Ghost Base Offset", 
                        default_value=state.show_ghost_base_offset,
                        callback=toggle_ghost_base_offset
                    )
                    help_text = dpg.add_text("(?)", color=(200, 200, 100))
                    with dpg.tooltip(help_text):
                        dpg.add_text(
                            "Shows transparent axis arrows on top of the sensor that represent \n"
                            "where the axes will position themselves when 'Set Offset' is pressed. \n"
                            "Setting the base offset will change the rotation of those ghost axes.",
                            wrap=300
                        )
                
                # Show Ghost Base Tare
                with dpg.group(horizontal=True):
                    dpg.add_checkbox(
                        label="Show Ghost Base Tare", 
                        default_value=state.show_ghost_base_tare,
                        callback=toggle_ghost_base_tare
                    )
                    help_text = dpg.add_text("(?)", color=(200, 200, 100))
                    with dpg.tooltip(help_text):
                        dpg.add_text(
                            "Shows a ghost camera orbiting with the sensor and represents where \n"
                            "the camera will go when 'Set Tare' is pressed. Setting the base tare \n"
                            "will change how the ghost camera is anchored to the sensor.",
                            wrap=300
                        )
                
                # Show Tare Camera
                with dpg.group(horizontal=True):
                    dpg.add_checkbox(
                        label="Show Tare Camera", 
                        default_value=state.show_tare_camera,
                        callback=toggle_tare_camera
                    )
                    help_text = dpg.add_text("(?)", color=(200, 200, 100))
                    with dpg.tooltip(help_text):
                        dpg.add_text(
                            "Toggles the visibility of the opaque camera that represents the PoV \n"
                            "of the Tare view on the right.",
                            wrap=300
                        )
                
                dpg.add_separator()
                
                # Auto Base Tare on Offset
                with dpg.group(horizontal=True):
                    dpg.add_checkbox(
                        label="Auto Base Tare on Offset", 
                        default_value=sensor.tare_auto_base,
                        callback=toggle_tare_auto_base
                    )
                    help_text = dpg.add_text("(?)", color=(200, 200, 100))
                    with dpg.tooltip(help_text):
                        dpg.add_text(
                            "If enabled, upon doing 'Set Offset', the base tare will automatically \n"
                            "be set such that taring the camera will cause the camera axes and \n"
                            "offset axes to align. This is especially noticeable when 'Offset \n"
                            "Rotates Model' is checked as it makes it so taring after offset \n"
                            "always results in the identity orientation. If not checked, the base \n"
                            "tare is unaffected by setting the offset, and taring may no longer \n"
                            "result in the identity orientation after an offset is applied.",
                            wrap=300
                        )
            
            # Axis order controls
            with dpg.table_cell():
                dpg.add_text("AXIS ORDER", color=(255, 200, 255))
                
                # Permutation dropdown with tooltip
                with dpg.group(horizontal=True):
                    dpg.add_text("Permutation:")
                    help_text = dpg.add_text("(?)", color=(200, 200, 100))
                    with dpg.tooltip(help_text):
                        dpg.add_text(
                            "WARNING: Changing axis order does NOT convert existing "
                            "offset/tare values to the new axis space.\n\n"
                            "This may cause the model/axes/camera to jump in position.\n\n"
                            "Best practice: Set axis order BEFORE applying offset/tare.",
                            wrap=300
                        )
                dpg.add_combo(
                    items=["xyz", "xzy", "yxz", "yzx", "zxy", "zyx"],
                    default_value=state.axis_permutation,
                    callback=on_axis_permutation_change,
                    width=100
                )
                
                # Negation checkboxes - horizontal layout
                dpg.add_text("Negations:")
                with dpg.group(horizontal=True):
                    dpg.add_checkbox(
                        label="X",
                        default_value=state.negate_x,
                        callback=on_negate_x_change
                    )
                    dpg.add_checkbox(
                        label="Y",
                        default_value=state.negate_y,
                        callback=on_negate_y_change
                    )
                    dpg.add_checkbox(
                        label="Z",
                        default_value=state.negate_z,
                        callback=on_negate_z_change
                    )
                
                # Display current axis order
                dpg.add_spacer(height=5)
                dpg.add_text("axis_order: xyz / ENU", tag="axis_order_text", color=(150, 255, 150))
            
            # Euler angle offset section
            with dpg.table_cell():
                dpg.add_text("EULER EDITOR", color=(255, 200, 150))
                
                # Dropdowns side by side
                with dpg.group(horizontal=True):
                    with dpg.group():
                        dpg.add_text("Order:")
                        dpg.add_combo(
                            items=["XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX", 
                                   "XYX", "XZX", "YXY", "YZY", "ZXZ", "ZYZ"],
                            default_value=state.euler_order,
                            callback=on_euler_order_change,
                            width=70
                        )
                    
                    dpg.add_spacer(width=10)
                    
                    with dpg.group():
                        dpg.add_text("Type:")
                        dpg.add_combo(
                            items=["Offset", "Tare", "Base Offset", "Base Tare", "Untared"],
                            default_value=state.euler_type,
                            callback=on_euler_type_change,
                            width=100
                        )
                
                # Rotation inputs (labels will be updated based on order)
                dpg.add_text("Angles (deg):")
                dpg.add_drag_float(label="X", tag="euler_angle_0", default_value=0.0, 
                                   callback=on_euler_angle_0_change, width=120, speed=0.5,
                                   max_value=180, min_value=-180)
                dpg.add_drag_float(label="Y", tag="euler_angle_1", default_value=0.0, 
                                   callback=on_euler_angle_1_change, width=120, speed=0.5,
                                   max_value=180, min_value=-180)
                dpg.add_drag_float(label="Z", tag="euler_angle_2", default_value=0.0, 
                                   callback=on_euler_angle_2_change, width=120, speed=0.5,
                                   max_value=180, min_value=-180)
                
                # Extrinsic checkbox with tooltip
                with dpg.group(horizontal=True):
                    dpg.add_checkbox(
                        label="Extrinsic",
                        default_value=state.euler_extrinsic,
                        callback=on_euler_extrinsic_change
                    )
                    help_text = dpg.add_text("(?)", color=(200, 200, 100))
                    with dpg.tooltip(help_text):
                        dpg.add_text(
                            "Depending on what type is selected (such as Base Offset), extrinsic "
                            "and intrinsic may appear to produce results opposite of what would be "
                            "expected. This occurs because some of the rotations are already in "
                            "global space, and some rotation results are displayed via inverting "
                            "the quaternion.",
                            wrap=300
                        )
            
            # Sensor data display
            with dpg.table_cell():
                dpg.add_text("SENSOR DATA", color=(150, 200, 255))
                with dpg.table(header_row=True, borders_innerH=True, borders_innerV=True):
                    dpg.add_table_column(label="Type", width_fixed=True, init_width_or_weight=40)
                    dpg.add_table_column(label="X", width_fixed=True, init_width_or_weight=50)
                    dpg.add_table_column(label="Y", width_fixed=True, init_width_or_weight=50)
                    dpg.add_table_column(label="Z", width_fixed=True, init_width_or_weight=50)
                    
                    # Accelerometer row
                    with dpg.table_row():
                        dpg.add_text("Accel")
                        dpg.add_text("  0.00", tag="accel_x")
                        dpg.add_text("  0.00", tag="accel_y")
                        dpg.add_text("  0.00", tag="accel_z")
                    
                    # Gyroscope row
                    with dpg.table_row():
                        dpg.add_text("Gyro")
                        dpg.add_text("  0.00", tag="gyro_x")
                        dpg.add_text("  0.00", tag="gyro_y")
                        dpg.add_text("  0.00", tag="gyro_z")
                    
                    # Magnetometer row
                    with dpg.table_row():
                        dpg.add_text("Mag")
                        dpg.add_text("  0.00", tag="mag_x")
                        dpg.add_text("  0.00", tag="mag_y")
                        dpg.add_text("  0.00", tag="mag_z")
                
                # Axis offset enabled checkbox with tooltip
                dpg.add_spacer(height=5)
                with dpg.group(horizontal=True):
                    dpg.add_checkbox(
                        label="Axis Offset Enabled",
                        default_value=sensor.axis_offset_enabled,
                        callback=toggle_axis_offset_enabled
                    )
                    help_text = dpg.add_text("(?)", color=(200, 200, 100))
                    with dpg.tooltip(help_text):
                        dpg.add_text(
                            "When enabled, component data (accel/gyro/mag) is mapped to the "
                            "axis space shown in the untared/offset scene view on the left.\n\n"
                            "When disabled, component data remains in the sensor's native axis space "
                            "based on the current axis order. This may be preferable in some situations" \
                            " to maintain consistency with the sensor's physical component axes.",
                            wrap=300
                        )

dpg.set_primary_window("primary_window", True)

# ==============================================================================
# MAIN LOOP
# ==============================================================================
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.configure_app(manual_callback_management=True)

# Initialize axis order display
update_axis_order()

# Start sensor streaming
sensor.startStreaming()

print("Application running. Use buttons to set offset/tare.")
print("Press Ctrl+C or close window to exit.")

try:
    while dpg.is_dearpygui_running():
        # Update sensor data
        sensor.updateStreaming()
        packet = sensor.getNewestStreamingPacket()
        sensor.clearStreamingPackets()
        
        # Always render if in Euler mode, or if we have a packet
        should_render = packet is not None
        
        if should_render:
            # Skip this packet if axis order was just changed
            # To prevent rendering outdated packet with mismatched axis order
            if state.skip_next_packet:
                state.skip_next_packet = False
                continue
            
            untared_orientation = packet.data[0]  # Keep sensor updated
            accel = packet.data[1]
            gyro = packet.data[2]
            mag = packet.data[3]

            if sensor.use_real_data and state.euler_type == "Untared":
                # If using real sensor data for untared, update Euler editor
                load_euler_from_quat()

            # Update sensor data display with fixed-width formatting
            dpg.set_value("accel_x", f"{accel[0]:7.2f}")
            dpg.set_value("accel_y", f"{accel[1]:7.2f}")
            dpg.set_value("accel_z", f"{accel[2]:7.2f}")
            
            dpg.set_value("gyro_x", f"{gyro[0]:7.2f}")
            dpg.set_value("gyro_y", f"{gyro[1]:7.2f}")
            dpg.set_value("gyro_z", f"{gyro[2]:7.2f}")
            
            dpg.set_value("mag_x", f"{mag[0]:7.2f}")
            dpg.set_value("mag_y", f"{mag[1]:7.2f}")
            dpg.set_value("mag_z", f"{mag[2]:7.2f}")
            
            # Convert from sensor space to OpenGL space
            gl_untared = state.axis_order.swap_to(GL_AXIS_ORDER, untared_orientation, rotational=True)
            gl_tare = state.axis_order.swap_to(GL_AXIS_ORDER, sensor.tare_quat, rotational=True)
            gl_offset = state.axis_order.swap_to(GL_AXIS_ORDER, sensor.offset_quat, rotational=True)
            gl_base_offset = state.axis_order.swap_to(GL_AXIS_ORDER, sensor.base_offset_quat, rotational=True)
            gl_base_tare = state.axis_order.swap_to(GL_AXIS_ORDER, sensor.base_tare_quat, rotational=True)
            
            # Update ghost base offset axes rotation (inverse to show where base offset would move from)
            ghost_base_offset_axes.set_rotation_quat(quat_inverse(gl_base_offset))
            
            # Update ghost base tare camera (inverse, relative to model)
            #ghost_rotated_position = quat_rotate_vec(quat_inverse(gl_base_tare), ghost_base_tare_camera.base_offset)
            ghost_rotated_position = quat_rotate_vec(gl_base_tare, ghost_base_tare_camera.base_offset)
            ghost_base_tare_camera.set_position(ghost_rotated_position)
            ghost_base_tare_camera.set_rotation_quat(gl_base_tare)
            
            # Manually compute tared rotation: tare * untared * offset
            # (We compute it manually instead of using sensor's tared value for consistency)
            tared_quat = quat_mul(quat_mul(sensor.tare_quat, untared_orientation), sensor.offset_quat)
            gl_tared = state.axis_order.swap_to(GL_AXIS_ORDER, tared_quat, rotational=True)
            
            # Update offset scene based on visualization mode
            untared_orientation_gameobject.set_rotation_quat(gl_untared)
            if state.show_offset_on_model:
                # Offset rotates entire model, axes follow with no additional rotation
                offset_model.set_rotation_quat(gl_offset)
                offset_scene.axes.set_rotation_quat([0, 0, 0, 1])  # Identity - axes follow model
            else:
                # Model shows untared, axes get additional offset rotation
                offset_model.set_rotation_quat([0, 0, 0, 1])
                offset_scene.axes.set_rotation_quat(gl_offset)  # Axes rotated by offset (on top of model rotation)
                offset_scene.axes.set_rotation_quat(gl_offset)  
            
            # Update camera to show tare position (both rotation and orbital position)
            # Rotate the base offset position by inverse of tare to keep camera behind sensor
            rotated_position = quat_rotate_vec(quat_inverse(gl_tare), offset_camera.base_offset)
            offset_camera.set_position(rotated_position)
            
            # Apply inverse tare rotation to camera so it points at sensor
            offset_camera.set_rotation_quat(quat_inverse(gl_tare))
            
            # Update tared scene based on visualization mode
            if state.show_offset_on_model:
                # Offset rotates entire model, axes follow with no additional rotation
                tared_model.set_rotation_quat(gl_tared)
                tared_scene.axes.set_rotation_quat([0, 0, 0, 1])  # Identity - axes follow model
            else:
                # Model shows tare * untared, axes get additional offset rotation
                tare_untared = quat_mul(sensor.tare_quat, untared_orientation)
                gl_tare_untared = state.axis_order.swap_to(GL_AXIS_ORDER, tare_untared, rotational=True)
                tared_model.set_rotation_quat(gl_tare_untared)
                tared_scene.axes.set_rotation_quat(gl_offset)  # Axes rotated by offset (on top of model rotation)
        
        # Update camera positions and render
        offset_scene.update_camera_pos()
        offset_scene.update_camera_rotation()
        offset_dpg_scene.render()
        offset_dpg_scene.update_dpg_texture()
        
        tared_scene.update_camera_pos()
        tared_scene.update_camera_rotation()
        tared_dpg_scene.render()
        tared_dpg_scene.update_dpg_texture()
        
        # Render DearPyGui frame
        dpg.run_callbacks(dpg.get_callback_queue())
        dpg.render_dearpygui_frame()

except KeyboardInterrupt:
    print("\nShutting down...")

finally:
    # Cleanup
    sensor.stopStreaming()
    sensor.cleanup()
    dpg.destroy_context()
    GL_Context.cleanup()
    print("Application closed.")
