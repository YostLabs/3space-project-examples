from yostlabs.math import quaternion as yl_quat
from yostlabs.math.axes import AxisOrder
import enum
from typing import TypeVar, Generic

class BoneMode(enum.Enum):
    STATIC_GLOBAL = 0
    STATIC_LOCAL = 1

class Bone:

    MODE_STATIC_GLOBAL = 0
    MODE_STATIC_LOCAL = 1

    def __init__(self, mode: BoneMode = BoneMode.STATIC_LOCAL, parent: "Bone" = None):
        self.__global_rotation = [0, 0, 0, 1]
        self.__local_rotation = [0, 0, 0, 1]

        self.axis_order: AxisOrder = None

        self.__local_rotation_dirty = False
        self.__global_rotation_dirty = False

        self.parent: Bone = None
        self.children: set[Bone] = set()

        self.set_parent(parent)

        self.__mode = mode
    
    @property
    def global_rotation(self):
        if self.__global_rotation_dirty:
            #Must update the global rotation based on the local rotation and the parent's global rotation
            self.update_global_rotation_from_local()
        
        return self.__global_rotation
    
    @global_rotation.setter
    def global_rotation(self, value):
        self.__global_rotation = value
        if self.__mode == BoneMode.STATIC_LOCAL:
            #When manually setting the global rotation of a bone
            #that is in local mode, must immediately compute the local rotation
            #based on the current pose of the skeleton. This makes it so that
            #setting global is really equivalent to setting local.
            self.update_local_rotation_from_global()
        else:
            self.set_dirty()

    @property
    def local_rotation(self):
        if self.__local_rotation_dirty:
            #Must update the local rotation
            self.update_local_rotation_from_global()
        return self.__local_rotation
    
    @local_rotation.setter
    def local_rotation(self, value):
        self.__local_rotation = value
        if self.__mode == BoneMode.STATIC_GLOBAL:
            #When manually setting the local rotation of a bone
            #that is in global mode, must immediately compute the global rotation
            #based on the current pose of the skeleton. This makes it so that
            #setting local is really equivalent to setting global.
            self.update_global_rotation_from_local()
        else:
            self.set_dirty()

    def update_local_rotation_from_global(self):
        parent_global = self.parent.global_rotation if self.parent else [0, 0, 0, 1]
        self.__local_rotation = yl_quat.quat_mul(yl_quat.quat_inverse(parent_global), self.__global_rotation)
        self.__local_rotation_dirty = False

    def update_global_rotation_from_local(self):
        #Must update the global rotation based on the local rotation and the parent's global rotation
        parent_global = self.parent.global_rotation if self.parent else [0, 0, 0, 1]
        self.__global_rotation = yl_quat.quat_mul(parent_global, self.__local_rotation)
        self.__global_rotation_dirty = False

    def set_dirty(self):
        """
        Marks this bone and all its childeren as dirty
        to recompute their dependent rotations (Opposite of thier current mode)
        on the next access.
        """
        if self.__mode == BoneMode.STATIC_GLOBAL:
            self.__local_rotation_dirty = True
        elif self.__mode == BoneMode.STATIC_LOCAL:
            self.__global_rotation_dirty = True
        
        for child in self.children:
            child.set_dirty()

    #---------------------HIERARCHY LOGIC---------------------

    def add_child(self, child: "Bone"):
        if child in self.children: #Already a child, do nothing
            return
        
        # Remove previous parent relationship if exists
        if child.parent is not None:
            child.parent.children.remove(child)
        
        #Update new parent relationship
        self.children.add(child)
        child.parent = self
    
    def remove_child(self, child: "Bone"):
        if child not in self.children: #Not a child, do nothing
            return

        #Remove child relationship
        self.children.remove(child)
        child.parent = None

    def set_parent(self, parent: "Bone"):
        if self.parent is parent: #Already has this parent, do nothing
            return
        
        # Remove from old parent's children list
        if self.parent is not None:
            self.parent.children.remove(self)

        # Set new parent
        self.parent = parent
        if parent is not None:
            parent.children.add(self)
    
    @property
    def mode(self):
        return self.__mode
    
    @mode.setter
    def mode(self, value: BoneMode):
        if self.__mode == value:
            return
        
        #Resolve the rotations before changing the mode to ensure
        #that the current pose of the skeleton is maintained
        if self.__local_rotation_dirty:
            self.update_local_rotation_from_global()
        if self.__global_rotation_dirty:
            self.update_global_rotation_from_local()

        self.__mode = value

    def clone(self, other: "Bone"):
        #Directly set the global and local rotations of this
        #and clear dirty flags
        self.__global_rotation = other.global_rotation
        self.__local_rotation = other.local_rotation
        self.__mode = other.mode
        self.__global_rotation_dirty = False
        self.__local_rotation_dirty = False

# Generic type variable for bone types
BoneT = TypeVar('BoneT', bound=Bone)

class Skeleton(Generic[BoneT]):
    """
    Represents a complete skeleton hierarchy.
    
    The Skeleton makes all bones accessible
    as attributes (e.g., skeleton.right_foot).

    By default, the pelvis is the root bone.
    """
    
    def __init__(self, bone_mode: BoneMode = BoneMode.STATIC_LOCAL, bone_class: type[BoneT] = Bone):
        """Create a complete skeleton with all bones.
        
        Args:
            bone_mode: The default mode for all bones (STATIC_GLOBAL or STATIC_LOCAL)
            bone_class: The bone class to use for creating all bones (defaults to Bone)
        """

        #Not used internally, but helpful for communicating to things that use the skeleton
        #what the expected axis order of the skeleton is for correctly orienting the model in 3D space.
        self.__axis_order = AxisOrder("NUE")
        
        self.pelvis: BoneT = bone_class(mode=bone_mode, parent=None)
        self.root: BoneT = self.pelvis  # Alias for pelvis as root bone

        self.torso: BoneT = bone_class(mode=bone_mode, parent=self.pelvis)
        self.neck: BoneT = bone_class(mode=bone_mode, parent=self.torso)
        self.head: BoneT = bone_class(mode=bone_mode, parent=self.neck)

        self.left_upper_arm: BoneT = bone_class(mode=bone_mode, parent=self.torso)
        self.left_lower_arm: BoneT = bone_class(mode=bone_mode, parent=self.left_upper_arm)
        self.left_hand: BoneT = bone_class(mode=bone_mode, parent=self.left_lower_arm)

        self.right_upper_arm: BoneT = bone_class(mode=bone_mode, parent=self.torso)
        self.right_lower_arm: BoneT = bone_class(mode=bone_mode, parent=self.right_upper_arm)
        self.right_hand: BoneT = bone_class(mode=bone_mode, parent=self.right_lower_arm)

        self.left_upper_leg: BoneT = bone_class(mode=bone_mode, parent=self.pelvis)
        self.left_lower_leg: BoneT = bone_class(mode=bone_mode, parent=self.left_upper_leg)
        self.left_kneecap: BoneT = bone_class(mode=bone_mode, parent=self.left_lower_leg)
        self.left_foot: BoneT = bone_class(mode=bone_mode, parent=self.left_lower_leg)

        self.right_upper_leg: BoneT = bone_class(mode=bone_mode, parent=self.pelvis)
        self.right_lower_leg: BoneT = bone_class(mode=bone_mode, parent=self.right_upper_leg)
        self.right_kneecap: BoneT = bone_class(mode=bone_mode, parent=self.right_lower_leg)
        self.right_foot: BoneT = bone_class(mode=bone_mode, parent=self.right_lower_leg)

        # Propagate the default axis order to all bones
        for bone in self.get_all_bones():
            bone.axis_order = self.__axis_order
    
    @property
    def axis_order(self):
        """Get the axis order of the skeleton, which represents the expected facing direction and up direction of the model."""
        return self.__axis_order
    
    @axis_order.setter
    def axis_order(self, value: AxisOrder):
        """Set the axis order of the skeleton, which represents the expected facing direction and up direction of the model."""
        self.__axis_order = value
        for bone in self.get_all_bones():
            bone.axis_order = value

    def get_all_bones(self) -> list[BoneT]:
        """Get all bone objects in the skeleton."""
        return [
            self.pelvis,
            self.torso,
            self.neck,
            self.head,
            self.left_upper_arm,
            self.left_lower_arm,
            self.left_hand,
            self.right_upper_arm,
            self.right_lower_arm,
            self.right_hand,
            self.left_upper_leg,
            self.left_lower_leg,
            self.left_kneecap,
            self.left_foot,
            self.right_upper_leg,
            self.right_lower_leg,
            self.right_kneecap,
            self.right_foot
        ]
    
    @classmethod
    def from_skeleton(cls, source: "Skeleton[Bone]") -> "Skeleton[BoneT]":
        """ 
        Create a new skeleton from an existing skeleton, copying all bone rotations.
        
        This classmethod works with any Skeleton subclass - when called on ThreespaceSkeleton,
        it returns a ThreespaceSkeleton with the source's pose copied.
        
        Args:
            source: The source Skeleton to copy rotations from
            
        Returns:
            A new skeleton instance of the same type as cls, with rotations copied
        """
        # Create a new skeleton of the calling class type
        new_skeleton: Skeleton[BoneT] = cls()
        
        # Copy rotations from source skeleton to corresponding bones
        dst_bones: list[BoneT] = new_skeleton.get_all_bones()
        for src_bone, dst_bone in zip(source.get_all_bones(), dst_bones):
            dst_bone.clone(src_bone)
        
        return new_skeleton        
