import glfw
import dearpygui.dearpygui as dpg

from yostlabs.graphics import GL_Context
from yostlabs.graphics.scene_prefabs import CameraScene
from yostlabs.graphics.glfw import GlfwCameraMover
import yostlabs.math.quaternion as yl_quat

from skeleton import Skeleton, BoneMode
from skeleton_graphics import VisualSkeleton
from joint_analyzer import JointAnalyzer, Joint

TEXTURE_WIDTH = 800
TEXTURE_HEIGHT = 800


# ==================== SETUP 3D VIEWER ====================
GL_Context.init(
    window_width=TEXTURE_WIDTH,
    window_height=TEXTURE_HEIGHT,
    visible=True,
    window_title="Joint Analyzer Example",
)
window = GL_Context.get_window()

scene = CameraScene(TEXTURE_WIDTH, TEXTURE_HEIGHT, name="Joint Analysis Scene")
scene.camera.set_position([0, 0, 80])
camera_control = GlfwCameraMover(scene.camera, window, camera_speed=30)
scene.set_background_color(0.2, 0.2, 0.25, 1.0)

skeleton = Skeleton(bone_mode=BoneMode.STATIC_LOCAL)
visual_skeleton = VisualSkeleton(skeleton)
scene.add_child(visual_skeleton)

visual_skeleton.set_rotation_quat(yl_quat.quat_from_euler([180], "y", degrees=True))
visual_skeleton.set_tpose()
visual_skeleton.show_all_axes()

# ==================== JOINT ANALYZER ====================
analyzer = JointAnalyzer(skeleton)
all_joints = analyzer.get_all_joints()
joints_by_name = {joint.primary_name: joint for joint in all_joints}

# Compute once so slider defaults come from current pose
analyzer.compute_angles()


def update_joint_from_slider(sender, app_data, user_data):
    joint_name, axis_index = user_data
    joint = joints_by_name.get(joint_name)
    if joint is None:
        return

    joint.set_angle(axis_index, app_data)
    visual_skeleton.update_pose()


def refresh_sliders_from_joints():
    analyzer.compute_angles()
    for joint_name, joint in joints_by_name.items():
        for i in range(3):
            tag = f"{joint_name}_{i}"
            if dpg.does_item_exist(tag):
                dpg.set_value(tag, joint.get_angle(i))


def reset_all_joints():
    visual_skeleton.set_tpose()
    refresh_sliders_from_joints()


def recompute_angles_from_model():
    # Snapshot previous angles before recomputing
    previous = {
        joint.primary_name: [joint.get_angle(i) for i in range(3)]
        for joint in all_joints
    }

    analyzer.compute_angles()

    for joint in all_joints:
        joint_name = joint.primary_name
        prev_angles = previous[joint_name]
        new_angles = [joint.get_angle(i) for i in range(3)]
        if any(abs(new - old) > 0.01 for new, old in zip(new_angles, prev_angles)):
            print(
                f"WARNING: {joint_name} changed: "
                f"{prev_angles} -> {new_angles} "
                f"(delta: {[new - old for new, old in zip(new_angles, prev_angles)]})"
            )

        for i in range(3):
            tag = f"{joint_name}_{i}"
            if dpg.does_item_exist(tag):
                dpg.set_value(tag, new_angles[i])

def create_joint_sliders(parent, joint: Joint):
    joint_name = joint.primary_name
    display_name = joint_name.replace("_", " ").title()

    with dpg.collapsing_header(label=display_name, default_open=True, parent=parent):
        for i in range(3):
            raw_name = joint.get_axis_name(i)
            if '/' in raw_name:
                parts = raw_name.split('/', 1)
                raw_name = f"{parts[1].strip()}/{parts[0].strip()}"
            axis_name = raw_name.title()
            low, high = joint.ranges[i]
            dpg.add_text(f"{axis_name}:")
            with dpg.group(horizontal=True):
                dpg.add_slider_float(
                    tag=f"{joint_name}_{i}",
                    default_value=joint.get_angle(i),
                    min_value=low,
                    max_value=high,
                    width=380,
                    callback=update_joint_from_slider,
                    user_data=(joint_name, i),
                )
                dpg.add_text(f"[{low:.0f}, {high:.0f}]°", color=(160, 160, 160))

        dpg.add_spacer(height=5)


# ==================== SETUP DEARPYGUI WINDOW ====================
dpg.create_context()
dpg.create_viewport(title="Custom Joint Controller", width=575, height=725)

with dpg.window(label="Custom Joint Controller", tag="main_window", width=530, height=660, pos=(10, 10), no_close=True):
    dpg.add_text("Interactive Joint Controller (Custom Analyzer)", color=(100, 200, 255))
    dpg.add_separator()
    dpg.add_spacer(height=5)

    dpg.add_text("Adjust sliders to control currently implemented custom joints", color=(150, 150, 150))
    with dpg.group(horizontal=True):
        dpg.add_button(label="Reset All", callback=lambda: reset_all_joints(), width=100)
        dpg.add_button(label="Recompute Angles", callback=lambda: recompute_angles_from_model(), width=130)

    convention_text = dpg.add_text("(?) Axis name convention", color=(200, 200, 100))
    with dpg.tooltip(convention_text):
        dpg.add_text("Axis labels are shown as  Negative / Positive to match the sliders.\n"
                     "The left side of the slider (minimum) = negative direction\n"
                     "The right side of the slider (maximum) = positive direction",
                     color=(220, 220, 220))

    dpg.add_spacer(height=5)
    dpg.add_separator()
    dpg.add_spacer(height=5)

    with dpg.child_window(height=580, border=True, tag="joints_scroll"):
        for joint in all_joints:
            create_joint_sliders("joints_scroll", joint)

dpg.set_primary_window("main_window", True)

# Align sliders with model state
refresh_sliders_from_joints()

# Show DPG
dpg.setup_dearpygui()
dpg.show_viewport()

# ==================== MAIN RENDER LOOP ====================
while not glfw.window_should_close(window) and dpg.is_dearpygui_running():
    glfw.poll_events()

    camera_control.update_camera_pos()
    camera_control.update_camera_rotation()

    scene.render()
    glfw.swap_buffers(window)

    dpg.render_dearpygui_frame()

# Cleanup
GL_Context.cleanup()
dpg.destroy_context()
