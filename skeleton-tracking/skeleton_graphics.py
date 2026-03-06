from pathlib import Path

from skeleton import Skeleton, Bone, BoneMode
import numpy as np
from yostlabs.graphics import ModelObject, TransformNode, GL_AXIS_ORDER
from yostlabs.graphics.prefabs import TriAxesObject
from yostlabs.math import quaternion as yl_quat
from yostlabs.math.axes import AxisOrder

# Get the models directory
MODELS_DIR = Path(__file__).parent / "models"

def load_bone_model(bone_name: str) -> ModelObject:
    """Load a bone model from the models folder."""
    model_path = MODELS_DIR / f"{bone_name}.obj"
    if not model_path.exists():
        print(f"Warning: Model not found: {model_path}")
        return None
    return ModelObject(bone_name, model_path)

class VisualBone(TransformNode):
    """
    Represents a bone in the skeleton hierarchy.
    
    A VisualBone is a TransformNode with a mesh and optional axes visualization.
    The mesh is offset so the bone's pivot point is at the origin.
    """
    
    def __init__(self, name: str, mesh_name: str, positional_offset: list[float], rotational_offset: list[float] = [0, 0, 0, 1]):
        """
        Create a bone with mesh and axes.
        
        Args:
            name: Name of the bone
            mesh_name: Name of the mesh file to load (without path)
            positional_offset: [x, y, z] offset to apply to position the mesh relative to the bone's pivot
            rotational_offset: [x, y, z, w] quaternion to apply to rotate the mesh
        """
        super().__init__(name)

        self.mesh_joints: dict[str,TransformNode] = {}

        # Load and add mesh as child
        self.mesh_transform = TransformNode(f"{name}_MeshTransform")
        self.mesh_transform.set_rotation_quat(rotational_offset)
        self.mesh = load_bone_model(mesh_name)
        if self.mesh:
            self.mesh.set_position([-positional_offset[0], -positional_offset[1], -positional_offset[2]])
            # Rotate mesh to align with bone direction for facing
            self.mesh.set_rotation_quat(yl_quat.quat_from_euler([180], "y", degrees=True))  
            self.mesh_transform.add_child(self.mesh)
            self.add_child(self.mesh_transform)

        # Create axes (hidden by default)
        self.axes = TriAxesObject(f"{name}_Axes")
        #For now, just defaulting to the sensor space
        #No offsets being applied either to line up axes with individual
        #bones, just simply going to make an application that calibrates and then maps orientations
        #to bones on the skeleton
        self.axes.set_scale(4.8)  # Scale down axes a bit
        self.axes.set_active(False)  # Don't render axes by default
        self.add_child(self.axes)

    def add_joint(self, joint_name: str, position: list[float]):
        """Add a joint as a child of this bone."""
        joint = TransformNode(joint_name)
        joint.set_position(position)
        self.mesh_joints[joint_name] = joint
        self.mesh_transform.add_child(joint)
    
    def snap_bone_to_joint(self, joint_name: str, bone: TransformNode):
        """Attach an object to a joint on the bone."""
        if joint_name not in self.mesh_joints:
            print(f"Joint {joint_name} not found on bone {self.name}")
            return
        joint = self.mesh_joints[joint_name]
        position = joint.parent.rotation @ joint.position
        bone.set_position(position)

    def set_rotational_offset(self, offset_quat: list[float]):
        """Set a rotational offset for the bone's mesh."""
        self.mesh_transform.set_rotation_quat(offset_quat)

    def show_axes(self):
        """Show the bone's axes visualization."""
        self.axes.set_active(True)
    
    def hide_axes(self):
        """Hide the bone's axes visualization."""
        self.axes.set_active(False)
    
    def toggle_axes(self):
        """Toggle the bone's axes visualization."""
        self.axes.set_active(not self.axes.active)

    def set_axis_order(self, axis_order: AxisOrder):
        self.axes.set_axis_order(axis_order)

class VisualSkeleton(TransformNode):
    """
    A visual representation of a skeleton hierarchy, where each bone is represented as a 3D model.
    The visual skeleton is meant to be driven by an underlying Skeleton object that handles the actual
    pose and rotation logic. The VisualSkeleton simply applies the rotations from the Skeleton to the
    corresponding bone models for visualization.
    """
    
    def __init__(self, skeleton: Skeleton[Bone] = None):
        super().__init__("VisualSkeleton")

        """
        Create all bones and set up the skeleton hierarchy.
        Bones are created such that the skeleton is in a T-Pose facing into the screen/down the -Z axis.
        This involves all bones being rotated 180 degrees around the Y axis from their original orientation,
        in the model files which are in a Z-up forward facing orientation.
        """
        if skeleton is None:
            skeleton = Skeleton()

        self.backend = skeleton
        self.axis_order = None

        #Defining the position of the bone models pre any rotation to form the default T-Pose.
        #These positions are purely the default positions of the model in GL space and will
        #be modified for building the actual skeleton via the joint system on the Visual Bones.
        global_hip_pos = np.array([0, -3.9102, 0])
        global_torso_pos = global_hip_pos  # They share the same zero point on a joint
        global_neck_pos = np.array([0, 13.937, 1.9985])
        global_head_pos = np.array([0, 18.434, 1.375])

        global_left_upper_arm_pos = np.array([-6.406, 10.971, 1.4859])
        global_left_lower_arm_pos = np.array([-17.023, 11.269, 1.6873])
        global_left_hand_pos = np.array([-26.721, 10.536, 1.927])

        global_right_upper_arm_pos = np.array([6.406, 10.971, 1.4859])
        global_right_lower_arm_pos = np.array([17.023, 11.269, 1.6873])
        global_right_hand_pos = np.array([26.721, 10.536, 1.927])

        global_left_upper_leg_pos = np.array([-3.5091, -8.1999, 0])
        global_left_lower_leg_pos = np.array([-3.2943, -24.338, 1.6669])
        global_left_kneecap_pos = np.array([-3.2943, -24.338, 0.26982])
        global_left_foot_pos = np.array([-3.3007, -39.583, 3.5496])

        global_right_upper_leg_pos = np.array([3.5091, -8.1999, 0])
        global_right_lower_leg_pos = np.array([3.2943, -24.338, 1.6669])
        global_right_kneecap_pos = np.array([3.2943, -24.338, 0.26982])
        global_right_foot_pos = np.array([3.3007, -39.583, 3.5496])

        # Pelvis (root bone)
        self.pelvis = VisualBone("Pelvis", "pelvis", global_hip_pos)
        self.pelvis.add_joint("torso", global_torso_pos - global_hip_pos)
        self.pelvis.add_joint("left_upper_leg", global_left_upper_leg_pos - global_hip_pos)
        self.pelvis.add_joint("right_upper_leg", global_right_upper_leg_pos - global_hip_pos)
        self.add_child(self.pelvis)
        
        # Torso
        self.torso = VisualBone("Torso", "torso", global_torso_pos)
        self.torso.add_joint("neck", global_neck_pos - global_torso_pos)
        self.torso.add_joint("left_shoulder", global_left_upper_arm_pos - global_torso_pos)
        self.torso.add_joint("right_shoulder", global_right_upper_arm_pos - global_torso_pos)
        self.pelvis.add_child(self.torso)
        
        # Neck and Head
        self.neck = VisualBone("Neck", "neck", global_neck_pos)
        self.neck.add_joint("head", global_head_pos - global_neck_pos)
        self.torso.add_child(self.neck)
        self.torso.snap_bone_to_joint("neck", self.neck)
        
        self.head = VisualBone("Head", "skull", global_head_pos)
        self.head.set_position(global_head_pos - global_neck_pos)
        self.neck.add_child(self.head)
        self.neck.snap_bone_to_joint("head", self.head)
    
        # Left arm chain
        self.left_upper_arm = VisualBone("LeftUpperArm", "lupperarm", global_left_upper_arm_pos)
        self.left_upper_arm.set_rotational_offset(yl_quat.quat_from_euler([90], "z", degrees=True, extrinsic=True))
        self.left_upper_arm.add_joint("elbow", global_left_lower_arm_pos - global_left_upper_arm_pos)
        self.torso.add_child(self.left_upper_arm)
        self.torso.snap_bone_to_joint("left_shoulder", self.left_upper_arm)
        
        self.left_lower_arm = VisualBone("LeftLowerArm", "llowerarm", global_left_lower_arm_pos)
        self.left_lower_arm.set_rotational_offset(yl_quat.quat_from_euler([90, 90], "zy", degrees=True, extrinsic=True))
        self.left_lower_arm.add_joint("wrist", global_left_hand_pos - global_left_lower_arm_pos)
        self.left_upper_arm.add_child(self.left_lower_arm)
        self.left_upper_arm.snap_bone_to_joint("elbow", self.left_lower_arm)
        
        self.left_hand = VisualBone("LeftHand", "lhand", global_left_hand_pos)
        self.left_hand.set_rotational_offset(yl_quat.quat_from_euler([90, 90], "zy", degrees=True, extrinsic=True))
        self.left_lower_arm.add_child(self.left_hand)
        self.left_lower_arm.snap_bone_to_joint("wrist", self.left_hand)
        
        # Right arm chain
        self.right_upper_arm = VisualBone("RightUpperArm", "rupperarm", global_right_upper_arm_pos)
        self.right_upper_arm.set_rotational_offset(yl_quat.quat_from_euler([-90], "z", degrees=True, extrinsic=True))
        self.right_upper_arm.add_joint("elbow", global_right_lower_arm_pos - global_right_upper_arm_pos)
        self.torso.add_child(self.right_upper_arm)
        self.torso.snap_bone_to_joint("right_shoulder", self.right_upper_arm)
    
        self.right_lower_arm = VisualBone("RightLowerArm", "rlowerarm", global_right_lower_arm_pos)
        self.right_lower_arm.set_rotational_offset(yl_quat.quat_from_euler([-90, -90], "zy", degrees=True, extrinsic=True))
        self.right_lower_arm.add_joint("wrist", global_right_hand_pos - global_right_lower_arm_pos)
        self.right_upper_arm.add_child(self.right_lower_arm)
        self.right_upper_arm.snap_bone_to_joint("elbow", self.right_lower_arm)
        
        self.right_hand = VisualBone("RightHand", "rhand", global_right_hand_pos)
        self.right_hand.set_rotational_offset(yl_quat.quat_from_euler([-90, -90], "zy", degrees=True, extrinsic=True))
        self.right_lower_arm.add_child(self.right_hand)
        self.right_lower_arm.snap_bone_to_joint("wrist", self.right_hand)
        
        # Left leg chain
        self.left_upper_leg = VisualBone("LeftUpperLeg", "lupperleg", global_left_upper_leg_pos)
        self.left_upper_leg.add_joint("knee", global_left_lower_leg_pos - global_left_upper_leg_pos)
        self.pelvis.add_child(self.left_upper_leg)
        self.pelvis.snap_bone_to_joint("left_upper_leg", self.left_upper_leg)
        
        self.left_lower_leg = VisualBone("LeftLowerLeg", "llowerleg", global_left_lower_leg_pos)
        self.left_lower_leg.add_joint("knee_cap", global_left_kneecap_pos - global_left_lower_leg_pos)
        self.left_lower_leg.add_joint("ankle", global_left_foot_pos - global_left_lower_leg_pos)
        self.left_upper_leg.add_child(self.left_lower_leg)
        self.left_upper_leg.snap_bone_to_joint("knee", self.left_lower_leg)
        
        self.left_kneecap = VisualBone("LeftKneecap", "lkneecap", global_left_kneecap_pos)
        self.left_lower_leg.add_child(self.left_kneecap)
        self.left_lower_leg.snap_bone_to_joint("knee_cap", self.left_kneecap)
        
        self.left_foot = VisualBone("LeftFoot", "lfoot", global_left_foot_pos)
        self.left_lower_leg.add_child(self.left_foot)
        self.left_lower_leg.snap_bone_to_joint("ankle", self.left_foot)
        
        # Right leg chain
        self.right_upper_leg = VisualBone("RightUpperLeg", "rupperleg", global_right_upper_leg_pos)
        self.right_upper_leg.add_joint("knee", global_right_lower_leg_pos - global_right_upper_leg_pos)
        self.pelvis.add_child(self.right_upper_leg)
        self.pelvis.snap_bone_to_joint("right_upper_leg", self.right_upper_leg)
        
        self.right_lower_leg = VisualBone("RightLowerLeg", "rlowerleg", global_right_lower_leg_pos)
        self.right_lower_leg.add_joint("knee_cap", global_right_kneecap_pos - global_right_lower_leg_pos)
        self.right_lower_leg.add_joint("ankle", global_right_foot_pos - global_right_lower_leg_pos)
        self.right_upper_leg.add_child(self.right_lower_leg)
        self.right_upper_leg.snap_bone_to_joint("knee", self.right_lower_leg)
        
        self.right_kneecap = VisualBone("RightKneecap", "rkneecap", global_right_kneecap_pos)
        self.right_lower_leg.add_child(self.right_kneecap)
        self.right_lower_leg.snap_bone_to_joint("knee_cap", self.right_kneecap)
        
        self.right_foot = VisualBone("RightFoot", "rfoot", global_right_foot_pos)
        self.right_lower_leg.add_child(self.right_foot)
        self.right_lower_leg.snap_bone_to_joint("ankle", self.right_foot)

        self.set_axis_order(skeleton.axis_order)
    
    def update_pose(self, skeleton: Skeleton[Bone] = None):
        """
        Updates the visual skeleton's pose based on the backend Skeleton's current pose.
        """
        backend = self.backend
        if skeleton is not None:
            backend = skeleton
        
        self.set_axis_order(backend.axis_order)
        
        self.pelvis.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.pelvis.local_rotation, rotational=True))
        self.torso.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.torso.local_rotation, rotational=True))
        self.neck.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.neck.local_rotation, rotational=True))
        self.head.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.head.local_rotation, rotational=True))

        self.left_upper_arm.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.left_upper_arm.local_rotation, rotational=True))
        self.left_lower_arm.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.left_lower_arm.local_rotation, rotational=True))
        self.left_hand.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.left_hand.local_rotation, rotational=True))

        self.right_upper_arm.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.right_upper_arm.local_rotation, rotational=True))
        self.right_lower_arm.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.right_lower_arm.local_rotation, rotational=True))
        self.right_hand.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.right_hand.local_rotation, rotational=True))

        self.left_upper_leg.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.left_upper_leg.local_rotation, rotational=True))
        self.left_lower_leg.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.left_lower_leg.local_rotation, rotational=True))
        self.left_kneecap.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.left_kneecap.local_rotation, rotational=True))
        self.left_foot.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.left_foot.local_rotation, rotational=True))

        self.right_upper_leg.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.right_upper_leg.local_rotation, rotational=True))
        self.right_lower_leg.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.right_lower_leg.local_rotation, rotational=True))
        self.right_kneecap.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.right_kneecap.local_rotation, rotational=True))
        self.right_foot.set_rotation_quat(self.axis_order.swap_to(GL_AXIS_ORDER, backend.right_foot.local_rotation, rotational=True))

    def show_all_axes(self):
        """Show axes for all bones in the skeleton."""
        for child in self._get_all_bones():
            child.show_axes()
    
    def hide_all_axes(self):
        """Hide axes for all bones in the skeleton."""
        for child in self._get_all_bones():
            child.hide_axes()
    
    def set_axis_order(self, axis_order: AxisOrder):
        """Set the axis order for all bones in the skeleton."""
        if axis_order == self.axis_order: return  # No change
        self.axis_order = axis_order
        for bone in self._get_all_bones():
            bone.set_axis_order(axis_order)

    def set_tpose(self):
        #Defined in Forward, Up, Right space, conver to current space
        order = AxisOrder("NUE")

        bone_to_mode_cache = {

        }

        #Revert all bones back to identity
        for bone in self.backend.get_all_bones():
            bone_to_mode_cache[bone] = bone.mode
            bone.mode = BoneMode.STATIC_LOCAL
            bone.local_rotation = [0, 0, 0, 1]

        #Make sure axis order is matching the backend skeleton for correct application of the T-Pose rotations
        self.set_axis_order(self.backend.axis_order)

        #Modify the few bones that are not identity in the T-Pose
        self.backend.right_upper_arm.local_rotation = order.swap_to(
            self.axis_order, yl_quat.quat_from_euler([-90], "x", degrees=True), 
            rotational=True)
        self.backend.right_lower_arm.local_rotation = order.swap_to(
            self.axis_order, yl_quat.quat_from_euler([90], "y", degrees=True), 
            rotational=True)

        self.backend.left_upper_arm.local_rotation = order.swap_to(
            self.axis_order, yl_quat.quat_from_euler([90], "x", degrees=True), 
            rotational=True)
        self.backend.left_lower_arm.local_rotation = order.swap_to(
            self.axis_order, yl_quat.quat_from_euler([-90], "y", degrees=True), 
            rotational=True)
        
        self.update_pose()

        #Swap bones back to their original modes
        for bone in self.backend.get_all_bones():
            bone.mode = bone_to_mode_cache[bone]

    def _get_all_bones(self) -> list[VisualBone]:
        """Get all bone objects in the skeleton."""
        return [self.pelvis, self.torso, self.neck, self.head,
                self.left_upper_arm, self.left_lower_arm, self.left_hand,
                self.right_upper_arm, self.right_lower_arm, self.right_hand,
                self.left_upper_leg, self.left_lower_leg, self.left_kneecap, self.left_foot,
                self.right_upper_leg, self.right_lower_leg, self.right_kneecap, self.right_foot]