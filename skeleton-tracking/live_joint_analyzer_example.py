"""
Creates a live ThreespaceSkeleton driven by connected sensors and displays the
computed JointAnalyzer angles in a Dear PyGui control panel.

This combines the sensor discovery / live skeleton update flow from
`sensor_skeleton_example.py` with the joint angle visualization UI from
`joint_analyzer_example.py`.
"""

import time

import dearpygui.dearpygui as dpg
import glfw

from yostlabs.communication.ble import ThreespaceBLEComClass
from yostlabs.graphics import GL_Context
from yostlabs.graphics.glfw import GlfwCameraMover
from yostlabs.graphics.scene_prefabs import CameraScene
from yostlabs.math.axes import AxisOrder
import yostlabs.math.quaternion as yl_quat
from yostlabs.tss3.api import ThreespaceSensor

from joint_analyzer import JointAnalyzer, Joint
from skeleton_graphics import VisualSkeleton
from threespace_skeleton import BoneMode, ThreespaceSkeleton, ThreespaceBone

TEXTURE_WIDTH = 800
TEXTURE_HEIGHT = 800
DISCOVERY_TIMEOUT_SECONDS = 20


def discover_sensors(sensor_map: dict[str, ThreespaceBone], timeout_seconds: float = DISCOVERY_TIMEOUT_SECONDS):
    discovered_coms: dict[str, ThreespaceBLEComClass] = {}

    print("Discovering nearby devices")
    ThreespaceBLEComClass.set_scanner_continous(True)
    try:
        start_time = time.perf_counter()
        while time.perf_counter() - start_time < timeout_seconds and len(discovered_coms) < len(sensor_map):
            for com in ThreespaceBLEComClass.auto_detect():
                if com.name not in sensor_map:
                    continue
                if com.name in discovered_coms:
                    continue
                discovered_coms[com.name] = com
                print(f"Discovered: {com.name}")
    finally:
        ThreespaceBLEComClass.set_scanner_continous(False)

    print("Done discovering.")
    print("Attempting to connect to:")
    print(list(discovered_coms.keys()))
    print()
    return discovered_coms


def connect_discovered_sensors(discovered_coms: dict[str, ThreespaceBLEComClass], sensor_map: dict[str, ThreespaceBone]):
    if "sk_pelvis" not in discovered_coms:
        raise RuntimeError("Failed to discover required pelvis sensor.")

    connected_names: list[str] = []
    failed_names: list[str] = []

    for com in discovered_coms.values():
        fail_count = 0
        success = False
        while not success and fail_count < 3:
            time.sleep(1)
            try:
                print(f"Attempting connection {fail_count + 1} {com.name}: ", end="")
                sensor = ThreespaceSensor(com, verbose=True)
                sensor_map[com.name].set_sensor(sensor)
                connected_names.append(com.name)
                print("Success!")
                success = True
            except Exception as exc:
                print(f"Failed :( {exc}")
                fail_count += 1

        if not success:
            failed_names.append(com.name)

    return connected_names, failed_names


def create_joint_slider(parent: str, joint: Joint):
    joint_name = joint.primary_name
    display_name = joint_name.replace("_", " ").title()

    with dpg.collapsing_header(label=display_name, default_open=True, parent=parent):
        for axis_index in range(3):
            raw_name = joint.get_axis_name(axis_index)
            if "/" in raw_name:
                parts = raw_name.split("/", 1)
                raw_name = f"{parts[1].strip()}/{parts[0].strip()}"
            axis_name = raw_name.title()
            low, high = joint.ranges[axis_index]

            dpg.add_text(f"{axis_name}:")
            with dpg.group(horizontal=True):
                dpg.add_slider_float(
                    tag=f"{joint_name}_{axis_index}",
                    default_value=joint.get_angle(axis_index),
                    min_value=low,
                    max_value=high,
                    width=380,
                    enabled=False,
                )
                dpg.add_text(f"[{low:.0f}, {high:.0f}]°", color=(160, 160, 160))

        dpg.add_spacer(height=5)


def refresh_sliders_from_joints(analyzer: JointAnalyzer, joints_by_name: dict[str, Joint]):
    analyzer.compute_angles()
    for joint_name, joint in joints_by_name.items():
        for axis_index in range(3):
            tag = f"{joint_name}_{axis_index}"
            if dpg.does_item_exist(tag):
                dpg.set_value(tag, joint.get_angle(axis_index))


def set_status_text(connected_names: list[str], failed_names: list[str]):
    status_lines = [f"Connected sensors ({len(connected_names)}): {', '.join(sorted(connected_names)) or 'None'}"]
    if failed_names:
        status_lines.append(f"Failed connections: {', '.join(sorted(failed_names))}")
    dpg.set_value("sensor_status", "\n".join(status_lines))


def main():
    dpg_context_created = False
    updating_started = False

    GL_Context.init(
        window_width=TEXTURE_WIDTH,
        window_height=TEXTURE_HEIGHT,
        visible=True,
        window_title="Live Joint Analyzer Example",
    )
    window = GL_Context.get_window()

    scene = CameraScene(TEXTURE_WIDTH, TEXTURE_HEIGHT, name="Live Joint Analysis Scene")
    scene.camera.set_position([0, 0, 80])
    camera_control = GlfwCameraMover(scene.camera, window, camera_speed=30)
    scene.set_background_color(0.2, 0.2, 0.25, 1.0)

    threespace_skeleton = ThreespaceSkeleton(bone_mode=BoneMode.STATIC_LOCAL)
    threespace_skeleton.axis_order = AxisOrder("NUE")

    visual_skeleton = VisualSkeleton(threespace_skeleton)
    scene.add_child(visual_skeleton)
    visual_skeleton.set_rotation_quat(yl_quat.quat_from_euler([180], "y", degrees=True))
    visual_skeleton.set_tpose()
    visual_skeleton.show_all_axes()

    analyzer = JointAnalyzer(threespace_skeleton)
    all_joints = analyzer.get_all_joints()
    joints_by_name = {joint.primary_name: joint for joint in all_joints}
    analyzer.compute_angles()

    sensor_map: dict[str,ThreespaceBone] = {
        "sk_head": threespace_skeleton.head,
        "sk_chest": threespace_skeleton.torso,
        "sk_pelvis": threespace_skeleton.pelvis,
        "sk_ru_leg": threespace_skeleton.right_upper_leg,
        "sk_lu_leg": threespace_skeleton.left_upper_leg,
        "sk_upper_arm": threespace_skeleton.right_upper_arm,
        "sk_lower_arm": threespace_skeleton.right_lower_arm,
        "sk_hand": threespace_skeleton.right_hand,
    }

    threespace_skeleton.pelvis.set_forward_axis("y")

    def recalibrate_sensors():
        print("Stand in skeleton pose to calibrate sensors...")
        visual_skeleton.set_tpose()
        scene.render()
        glfw.swap_buffers(window)
        time.sleep(2)
        print("Starting Calibration")
        threespace_skeleton.calibrate_sensors(rotate_root=False, tare_sensors=True)
        print("Done calibrating")

    try:
        discovered_coms = discover_sensors(sensor_map)
        connected_names, failed_names = connect_discovered_sensors(discovered_coms, sensor_map)

        dpg.create_context(manual_callback_management=True)
        dpg_context_created = True
        dpg.create_viewport(title="Live Joint Analyzer", width=575, height=760)

        with dpg.window(
            label="Live Joint Analyzer",
            tag="main_window",
            width=530,
            height=700,
            pos=(10, 10),
            no_close=True,
        ):
            dpg.add_text("Live Joint Analyzer (Sensor Driven)", color=(100, 200, 255))
            dpg.add_separator()
            dpg.add_spacer(height=5)
            dpg.add_text(
                "Sliders are updated every frame from JointAnalyzer while the ThreespaceSkeleton tracks the live sensors.",
                wrap=480,
                color=(150, 150, 150),
            )
            dpg.add_spacer(height=5)
            dpg.add_button(label="Recalibrate Sensors", callback=lambda: recalibrate_sensors(), width=160)
            dpg.add_spacer(height=5)
            dpg.add_text("", tag="sensor_status", wrap=480, color=(200, 200, 120))
            dpg.add_separator()
            dpg.add_spacer(height=5)

            with dpg.child_window(height=565, border=True, tag="joints_scroll"):
                for joint in all_joints:
                    create_joint_slider("joints_scroll", joint)

        dpg.set_primary_window("main_window", True)
        set_status_text(connected_names, failed_names)

        dpg.setup_dearpygui()
        dpg.show_viewport()

        recalibrate_sensors()
        refresh_sliders_from_joints(analyzer, joints_by_name)
        threespace_skeleton.start_updating()
        updating_started = True

        while not glfw.window_should_close(window) and dpg.is_dearpygui_running():
            glfw.poll_events()

            if glfw.get_key(window, glfw.KEY_R) == glfw.PRESS:
                recalibrate_sensors()

            camera_control.update_camera_pos()
            camera_control.update_camera_rotation()

            threespace_skeleton.update()
            visual_skeleton.update_pose()
            refresh_sliders_from_joints(analyzer, joints_by_name)

            scene.render()
            glfw.swap_buffers(window)
            dpg.run_callbacks(dpg.get_callback_queue())
            dpg.render_dearpygui_frame()
    finally:
        if updating_started:
            threespace_skeleton.stop_updating()
        GL_Context.cleanup()
        if dpg_context_created:
            dpg.destroy_context()


if __name__ == "__main__":
    main()
