"""
Creates a ThreespaceSkeleton, which is a skeleton that allows connecting sensors to the bones.
Demonstrates connecting to nearby sensors over BLE and assigning them to the skeleton. Handles
calibrating and updating the skeleton model based on the sensor data. Renders the skeleton
in a GLFW window with a simple camera setup.
"""

import time
import glfw

from yostlabs.graphics import GL_Context
from yostlabs.graphics.scene_prefabs import CameraScene
from yostlabs.graphics.glfw import GlfwCameraMover

import yostlabs.math.quaternion as yl_quat
from yostlabs.math.axes import AxisOrder

from yostlabs.tss3.api import ThreespaceSensor
from yostlabs.communication.ble import ThreespaceBLEComClass

from threespace_skeleton import ThreespaceSkeleton, BoneMode
from skeleton_graphics import VisualSkeleton

TEXTURE_WIDTH = 800
TEXTURE_HEIGHT = 800

#--------------------SETUP 3D VIEWER AND SKELETON-------------------

# SETUP 3D VIEWER
GL_Context.init(window_width=TEXTURE_WIDTH, window_height=TEXTURE_HEIGHT, visible=False, window_title="Threespace Skeleton Viewer")
window = GL_Context.get_window()

# Create a simple camera scene
scene = CameraScene(TEXTURE_WIDTH, TEXTURE_HEIGHT, name="Skeleton Scene")
scene.camera.set_position([0, 0, 80])  # Move camera back to view the skeleton
camera_control = GlfwCameraMover(scene.camera, window, camera_speed=30)
scene.set_background_color(0.2, 0.2, 0.25, 1.0)


# Create the skeleton, this represents the mathematical properties and orientation
threespace_skeleton = ThreespaceSkeleton(bone_mode=BoneMode.STATIC_LOCAL)

#Set the skeleton axis order to match the desired visual so that
#the sensors can be configured when assigned to match the axis order of the skeleton/bones
threespace_skeleton.axis_order = AxisOrder("NUE")

#Create the actual visualizer for the skeleton object
#To update this visual skeleton, modify the underlying skeleton
#and call update_pose() on the visual skeleton
visual_skeleton = VisualSkeleton(threespace_skeleton)
scene.add_child(visual_skeleton)

#-------------------DISCOVER NEARBY SENSORS-------------------

#Setup the BLE Name of the sensors to discover and their corresponding bones in the skeleton
sensor_map = {
    "sk_head": threespace_skeleton.head,
    "sk_chest": threespace_skeleton.torso,
    "sk_pelvis": threespace_skeleton.pelvis,
    "sk_ru_leg": threespace_skeleton.right_upper_leg,
    "sk_lu_leg": threespace_skeleton.left_upper_leg,
    "sk_upper_arm": threespace_skeleton.right_upper_arm,
    "sk_lower_arm": threespace_skeleton.right_lower_arm,
    "sk_hand": threespace_skeleton.right_hand,
}

discovered_coms: dict[str,ThreespaceBLEComClass] = {

}

print("Discovering nearby devices")
ThreespaceBLEComClass.set_scanner_continous(True)
start_time = time.perf_counter()
while time.perf_counter() - start_time < 20 and len(discovered_coms) < len(sensor_map):
    for com in ThreespaceBLEComClass.auto_detect():
        if com.name not in sensor_map: continue
        if com.name in discovered_coms: continue
        discovered_coms[com.name] = com
        print(f"Discovered: {com.name}")
ThreespaceBLEComClass.set_scanner_continous(False)

print("Done discovering.")
print("Attempting to connect to:")
print(list(discovered_coms.keys()))
print()

#--------------------CONNECT TO SENSORS AND ASSIGN TO BONES-------------------

if "sk_pelvis" not in discovered_coms:
    #This is only required because it is the sensor with a defined forward axis so the
    #skeleton knows which way forward is. Feel free to change this if you want to use
    #a different sensor.
    print("Failed to discover required pelvis, exiting...")
    exit()
threespace_skeleton.pelvis.set_forward_axis('y')

for com in discovered_coms.values():
    fail_count = 0
    success = False
    while not success and fail_count < 3:
        time.sleep(1)
        try:
            print(f"Attempting connection {fail_count+1} {com.name}: ", end="")
            sensor = ThreespaceSensor(com)
            sensor_map[com.name].set_sensor(sensor)
            print("Success!")
            success = True
        except Exception as e:
            print(f"Failed :( {e} {type(e)} {e.with_traceback()}")
            fail_count += 1

#---------------------CALIBRATION-------------------

def recalibrate_sensors():
    print("Stand in skeleton pose to calibrate sensors...")

    # Set visual T-pose for calibration
    visual_skeleton.set_tpose()
    scene.render()
    glfw.swap_buffers(window)

    time.sleep(2)
    print("Starting Calibration")
    # Taring to make the calibration position the default position
    threespace_skeleton.calibrate_sensors(rotate_root=False, tare_sensors=True)
    print("Done calibrating")

recalibrate_sensors()

#---------------------RENDER LOOP-------------------
visual_skeleton.set_rotation_quat(yl_quat.quat_from_euler([180], "y", degrees=True))
visual_skeleton.show_all_axes()

threespace_skeleton.start_updating()

# Main render loop
glfw.show_window(window)
while not glfw.window_should_close(window):
    glfw.poll_events()
    
    #This takes long enough to complete not need to check for the specif press
    if glfw.get_key(window, glfw.KEY_R) == glfw.PRESS:
        recalibrate_sensors()

    camera_control.update_camera_pos()
    camera_control.update_camera_rotation()

    threespace_skeleton.update()
    visual_skeleton.update_pose()

    # Render scene
    scene.render()
    glfw.swap_buffers(window)

threespace_skeleton.stop_updating()
GL_Context.cleanup()
