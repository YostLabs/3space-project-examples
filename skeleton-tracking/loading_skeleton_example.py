"""
Creates the mathematical skeleton construct and a visualizer for it.
Loads the skeleton into a GLFW window and demonstrates updating the pose.
"""

import glfw

from yostlabs.graphics import GL_Context
from yostlabs.graphics.scene_prefabs import CameraScene
from yostlabs.graphics.glfw import GlfwCameraMover

import yostlabs.math.quaternion as yl_quat
from skeleton import Skeleton, BoneMode
from skeleton_graphics import VisualSkeleton

TEXTURE_WIDTH = 800
TEXTURE_HEIGHT = 800

# SETUP 3D VIEWER
GL_Context.init(window_width=TEXTURE_WIDTH, window_height=TEXTURE_HEIGHT, visible=True, window_title="Threespace Skeleton Viewer")
window = GL_Context.get_window()

# Create a simple camera scene
scene = CameraScene(TEXTURE_WIDTH, TEXTURE_HEIGHT, name="Skeleton Scene")
scene.camera.set_position([0, 0, 80])  # Move camera back to view the skeleton
camera_control = GlfwCameraMover(scene.camera, window, camera_speed=30)
scene.set_background_color(0.2, 0.2, 0.25, 1.0)

# Create the skeleton, this represents the mathematical properties and orientation
skeleton = Skeleton(bone_mode=BoneMode.STATIC_LOCAL)

#Create the actual visualizer for the skeleton object
#To update this visual skeleton, modify the underlying skeleton
#and call update_pose() on the visual skeleton
visual_skeleton = VisualSkeleton(skeleton)
scene.add_child(visual_skeleton)

#Make model face viewer
visual_skeleton.set_rotation_quat(yl_quat.quat_from_euler([180], "y", degrees=True))
visual_skeleton.set_tpose()
visual_skeleton.show_all_axes()

# Main render loop
while not glfw.window_should_close(window):
    glfw.poll_events()

    camera_control.update_camera_pos()
    camera_control.update_camera_rotation()

    skeleton.right_upper_arm.local_rotation = yl_quat.quat_from_euler([90, -90], "zx", degrees=True)

    visual_skeleton.update_pose()

    # Render scene
    scene.render()
    glfw.swap_buffers(window)

GL_Context.cleanup()
