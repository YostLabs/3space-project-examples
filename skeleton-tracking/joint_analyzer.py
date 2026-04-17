#General: https://media.isbweb.org/images/documents/standards/Wu%20and%20Cavanagh%20J%20Biomech%2028%20(1995)%201258-1261.pdf
#Part 1 - Ankle, Hip, Spine: https://media.isbweb.org/images/documents/standards/Wu%20et%20al%20J%20Biomech%2035%20(2002)%20543%E2%80%93548.pdf
#Part 2 - Shoulder, Eblow, Wrist, Hand: https://media.isbweb.org/images/documents/standards/Wu%20et%20al%20J%20Biomech%2038%20(2005)%20981%E2%80%93992.pdf

#Note: This computes the joint angles based on orthogonal axes rather than the non-orthogonal
# axes proposed by Grood and Suntay. This is a close approximation that allows for more easily computing the angles,
# but for more clinically accurate angles, the Grood and Suntay method should be used.

from skeleton import Skeleton, Bone
from yostlabs.math import quaternion as yl_quat
from yostlabs.math.axes import AxisOrder
import math
from typing import Dict, Tuple, Optional
from enum import Enum

from dataclasses import dataclass, field

@dataclass
class Joint:
    
    # The order of Euler angle decomposition for this joint (e.g., "xyz", "zyx")
    bone: Bone

    #Name of the joint itself (e.g. "right_shoulder")
    primary_name: str
    #Name of the bone that the joint is based on (e.g. "right_upper_arm")
    secondary_name: str

    # The order of euler decomposition. Configurable to use the best decomp order
    # to avoid singularities for each joint.
    decomp_order: str = "zxy"
    axis_order: AxisOrder = AxisOrder("NUE")
    
    # Whether this joint is on the left side of the body (for angle sign conventions)
    left_side: bool = False

    # Human-readable names for each of the 3 axes, in the same order as decomp_order. Used for display purposes.
    # The names are in order of positive/negative.
    axis_names: Tuple[str, str, str] = ("flexion/extension", "adduction/abduction", "internal/external rotation")

    # Per-axis slider/display ranges in degrees
    ranges: Dict[int, Tuple[float, float]] = field(default_factory=lambda: {
        0: (-90.0, 90.0),
        1: (-90.0, 90.0),
        2: (-90.0, 90.0),
    })

    # Per-axis sign inversion for display/set values
    # True means the axis is negated relative to raw decomposed Euler angle.
    negate_axes: Dict[int, bool] = field(default_factory=lambda: {
        0: False,
        1: False,
        2: False,
    })

    #The computed joint angles in the order of the decomp_order
    angles: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    def __post_init__(self):
        if len(self.decomp_order) != 3:
            raise ValueError(f"decomp_order must be length 3, got '{self.decomp_order}'")
        if len(self.axis_names) != 3:
            raise ValueError("axis_names must have exactly 3 entries")
        if len(self.angles) != 3:
            self.angles = [0.0, 0.0, 0.0]

        # Convert any string keys in ranges/negate_axes to integer indices.
        # String keys are only valid for Tait-Bryan orders where the axis
        # characters 'z', 'x', 'y' map to flexion, adduction, and internal_rotation.
        # If the joint is not Tait-Bryan, an exception will occur when initializing.
        if self.is_tait_bryan:
            _str_key_to_index = {
                "flexion": self.flexion_index,
                "adduction": self.adduction_index,
                "internal_rotation": self.internal_rotation_index,
            }

            def _convert_keys(d: dict) -> dict:
                print(f"{self.primary_name} Converting:", d)
                result = {}
                for k, v in d.items():
                    if isinstance(k, int):
                        result[k] = v
                    else:
                        result[_str_key_to_index[k]] = v
                    
                return result

            self.ranges = _convert_keys(self.ranges)
            self.negate_axes = _convert_keys(self.negate_axes)

    def compute_angles(self):
        #Get bone rotation and convert to expected NUE axis order
        rotation = self.bone.local_rotation
        rotation = self.bone.axis_order.swap_to(self.axis_order, rotation, rotational=True)

        #Decompose into Euler Angles using the specified order
        self.angles = list(yl_quat.quat_to_euler_angles(rotation, self.decomp_order))
        self.angles = [math.degrees(a) for a in self.angles]

    @property
    def is_tait_bryan(self):
        order = self.decomp_order.lower()
        return set(order) == {"x", "y", "z"}

    def _require_tait_bryan(self):
        if not self.is_tait_bryan:
            raise ValueError(
                f"Joint '{self.primary_name}' uses '{self.decomp_order.upper()}', not Tait-Bryan angles; "
                "flexion/adduction/internal_rotation properties are unavailable. "
                "Use get_angle(index)/set_angle(index, value) instead."
            )

    def _axis_sign_by_index(self, index: int) -> float:
        """Return -1.0 if the axis at the given index is negated, else 1.0."""
        sign = 1
        if self.negate_axes[index]:
            sign = -1
        if self.left_side and self.decomp_order[index].lower() in ['x', 'y']:
            # For left side joints, also negate rotations that occur in the frontal and transverse planes 
            # (generally adduction and axial rotation) to match common biomechanical conventions
            sign *= -1
        return sign

    def get_axis_name(self, index: int) -> str:
        return self.axis_names[index]

    def semantic_name_to_index(self, name: str) -> Optional[int]:
        if name == "flexion":
            return self.flexion_index
        elif name == "adduction":
            return self.adduction_index
        elif name == "internal_rotation":
            return self.internal_rotation_index
        else:
            if name in self.axis_names:
                return self.axis_names.index(name)
            raise ValueError(f"Invalid axis name '{name}' for joint '{self.primary_name}'")

    def get_axis_range(self, index: int|str) -> Tuple[float, float]:
        if isinstance(index, str):
            index = self.semantic_name_to_index(index)
        return self.ranges[index]

    def is_axis_negated(self, index: int|str) -> bool:
        if isinstance(index, str):
            index = self.semantic_name_to_index(index)
        return self.negate_axes[index]

    def get_angle(self, index: int) -> float:
        return self.angles[index] * self._axis_sign_by_index(index)

    def set_angle(self, index: int, value: float):
        self.angles[index] = value * self._axis_sign_by_index(index)
        self.update_bone_rotation()

    @property
    def flexion_index(self):
        self._require_tait_bryan()
        return self.decomp_order.lower().index('z')
    
    @property
    def adduction_index(self):
        self._require_tait_bryan()
        return self.decomp_order.lower().index('x')
    
    @property
    def internal_rotation_index(self):
        self._require_tait_bryan()
        return self.decomp_order.lower().index('y')

    @property
    def flexion_name(self):
        self._require_tait_bryan()
        return self.axis_names[self.flexion_index]
    
    @property
    def adduction_name(self):
        self._require_tait_bryan()
        return self.axis_names[self.adduction_index]
    
    @property
    def internal_rotation_name(self):
        self._require_tait_bryan()
        return self.axis_names[self.internal_rotation_index]

    @property
    def flexion(self):
        return self.get_angle(self.flexion_index)

    @property
    def adduction(self):
        return self.get_angle(self.adduction_index)
    
    @property
    def internal_rotation(self):
        return self.get_angle(self.internal_rotation_index)
    
    @flexion.setter
    def flexion(self, value: float):
        self.set_angle(self.flexion_index, value)
    
    @adduction.setter
    def adduction(self, value: float):
        self.set_angle(self.adduction_index, value)
    
    @internal_rotation.setter
    def internal_rotation(self, value: float):
        self.set_angle(self.internal_rotation_index, value)

    def update_bone_rotation(self):
        #Recompose quaternion from Euler angles
        rotation = yl_quat.quat_from_euler(self.angles, self.decomp_order, degrees=True)

        #Swap back to the bone's actual axis order
        rotation = self.axis_order.swap_to(self.bone.axis_order, rotation, rotational=True)
        self.bone.local_rotation = rotation

class JointAnalyzer:
    """
    Analyzes joint angles from a Skeleton model.
    
    Converts bone quaternion rotations into anatomically-defined joint angles
    (flexion/extension, adduction/abduction, axial rotation) for biomechanical analysis.
    
    This analyzer works as a wrapper around any Skeleton instance and does not
    modify the skeleton structure.
    """
    
    def __init__(self, skeleton: Skeleton[Bone]):
        """
        Initialize the joint analyzer with a skeleton.
        
        Args:
            skeleton: The skeleton model to analyze
        """
        self.skeleton = skeleton
        self.axis_order = skeleton.axis_order

        # Spine / Axial
        self.head = Joint(bone=skeleton.head, left_side=False,
                          primary_name="head", secondary_name="atlanto_occipital",
                          axis_names=("flexion/extension", "lateral bending", "axial rotation"),
                          ranges={
                              "flexion": (-25.0, 25.0),
                              "adduction": (-10.0, 10.0),
                              "internal_rotation": (-8.0, 8.0),
                          },
                          negate_axes={
                              "flexion": True,
                              "adduction": False,
                              "internal_rotation": False,
                          })

        self.neck = Joint(bone=skeleton.neck, left_side=False,
                          primary_name="neck", secondary_name="C7/T1",
                          axis_names=("flexion/extension", "lateral bending", "axial rotation"),
                          ranges={
                              "flexion": (-45.0, 50.0),
                              "adduction": (-45.0, 45.0),
                              "internal_rotation": (-80.0, 80.0),
                          },
                          negate_axes={
                              "flexion": True,
                              "adduction": False,
                              "internal_rotation": False,
                          })

        self.lumbar = Joint(bone=skeleton.torso, left_side=False,
                            primary_name="lumbar", secondary_name="torso",
                            axis_names=("flexion/extension", "lateral bending", "axial rotation"),
                            ranges={
                                "flexion": (-30.0, 60.0),
                                "adduction": (-30.0, 30.0),
                                "internal_rotation": (-45.0, 45.0),
                            },
                            negate_axes={
                                "flexion": True,
                                "adduction": False,
                                "internal_rotation": False,
                            })

        # Left arm
        self.left_shoulder = Joint(bone=skeleton.left_upper_arm, left_side=True,
                                   primary_name="left_shoulder", secondary_name="left_upper_arm",
                                   decomp_order="yxy",
                                   axis_names=("Plane of Elevation", "Elevation", "internal/external rotation"),
                                   ranges={
                                       0: (-180.0, 180.0),
                                       1: (0.0, 180.0),
                                       2: (-90.0, 90.0),
                                   },
                                   negate_axes={
                                       0: False,
                                       1: True,
                                       2: False
                                   })

        self.left_elbow = Joint(bone=skeleton.left_lower_arm, left_side=True,
                                primary_name="left_elbow", secondary_name="left_lower_arm",
                                axis_names=("flexion/extension", "varus/valgus", "pronation"),
                                ranges={
                                    "flexion": (0.0, 150.0),
                                    "adduction": (-15.0, 15.0),
                                    "internal_rotation": (0, 180.0),
                                },
                                negate_axes={
                                    "flexion": False,
                                    "adduction": False,
                                    "internal_rotation": False,
                                })

        self.left_wrist = Joint(bone=skeleton.left_hand, left_side=True,
                                primary_name="left_wrist", secondary_name="left_hand",
                                axis_names=("flexion/extension", "ulnar/radial deviation", "pronation/supination"),
                                ranges={
                                    "flexion": (-70.0, 70.0),
                                    "adduction": (-20.0, 45.0),
                                    "internal_rotation": (-90.0, 90.0),
                                })

        # Right arm
        #Note: The right shoulder is more complicated than any other joint, even the other shoulder, because it uses proper euler
        #angles, and those decomp orders use acos for the middle angle, which has a range of 0-180. This means to rotate "backwards",
        #the other axes must first rotate to allow positive angles to go the opposite direction before then rotating back. The right
        #shoulder elevation with the default axis order of NUE means the elevation would be negative/backwards, and thus the computation 
        #causes the plane of elevation and axial rotation to go to 180 to allow the elevation to be positive. To fix this, we change the
        #mathematical axis order from NUE to SDE just for the right shoulder. This maintains the system as right handed, while allowing
        #the elevation to be naturally positive. This does cause the Up axis and to change sign, but this is fixed by simply negating
        #the resulting euler angles for the plane of elevation and axial rotation. The visual axis order expressed by the bone is still
        #NUE to be consistent with the rest of the model, it is only mathematically in the joint calculation that it is treated as SDE. 
        self.right_shoulder = Joint(bone=skeleton.right_upper_arm, left_side=False,
                                    primary_name="right_shoulder", secondary_name="right_upper_arm",
                                    decomp_order="yxy",
                                    axis_order=AxisOrder("SDE"),
                                    axis_names=("Plane of Elevation", "Elevation", "internal/external rotation"),
                                    ranges={
                                        0: (-180.0, 180.0),
                                        1: (0.0, 180.0),
                                        2: (-90.0, 90.0),
                                    },
                                    negate_axes={
                                        0: True,
                                        1: False,
                                        2: True
                                    })
        
        #You can try representing the shoulder the same as the other joints using ZXY decomp order, but this
        #will more frequently cause gimbal lock. And more importantly, the angles will be unstable whenever the 
        #adduction exceeds 90 degrees. This is due to the euler angle calculation using asin which has a range of 
        #-90 to 90, so when the arm is raised above 90 degrees, the other axes will flip to compensate, resulting 
        #in large angle changes in ranges that do not make sense. You could attempt to fix this by changing the 0 
        #position of the shoulder to be when t-posing, but even then angles can easily exceed -90 to 90. For these
        #reasons it is recommended to use the proper Euler angle decomposition for the shoulder.
        #Note: Even the proper euler angles will have a similar jump if the arm exceeds 180 degrees of elevation, 
        #but this is less common.
        # self.right_shoulder = Joint(bone=skeleton.right_upper_arm, left_side=False,
        #                             primary_name="right_shoulder", secondary_name="right_upper_arm",
        #                             decomp_order="ZXY",
        #                             axis_names=("Flexion/Extension", "Abduction/Adduction", "internal/external rotation"),
        #                             ranges={
        #                                 "flexion": (-180.0, 180.0),
        #                                 "adduction": (0.0, 180.0),
        #                                 "internal_rotation": (-90, 90.0),
        #                             },
        #                             negate_axes={
        #                                 "flexion": False,
        #                                 "adduction": True,
        #                                 "internal_rotation": False,
        #                             })        

        self.right_elbow = Joint(bone=skeleton.right_lower_arm, left_side=False,
                                 primary_name="right_elbow", secondary_name="right_lower_arm",
                                 axis_names=("flexion/extension", "varus/valgus", "pronation"),
                                 ranges={
                                     "flexion": (0.0, 150.0),
                                     "adduction": (-15.0, 15.0),
                                     "internal_rotation": (0, 180.0),
                                 },
                                 negate_axes={
                                     "flexion": False,
                                     "adduction": False,
                                     "internal_rotation": False,
                                 })

        self.right_wrist = Joint(bone=skeleton.right_hand, left_side=False,
                                 primary_name="right_wrist", secondary_name="right_hand",
                                 axis_names=("flexion/extension", "ulnar/radial deviation", "pronation/supination"),
                                 ranges={
                                     "flexion": (-70.0, 70.0),
                                     "adduction": (-20.0, 45.0),
                                     "internal_rotation": (-90.0, 90.0),
                                 })

        # Left leg
        self.left_hip = Joint(bone=skeleton.left_upper_leg, left_side=True,
                              primary_name="left_hip", secondary_name="left_upper_leg",
                              ranges={
                                  "flexion": (-30.0, 130.0),
                                  "adduction": (-60.0, 30.0),
                                  "internal_rotation": (-45.0, 45.0),
                              })

        self.left_knee = Joint(bone=skeleton.left_lower_leg, left_side=True,
                               primary_name="left_knee", secondary_name="left_lower_leg",
                               axis_names=("flexion/extension", "varus/valgus", "internal/external rotation"),
                               ranges={
                                   "flexion": (0.0, 140.0),
                                   "adduction": (-10.0, 10.0),
                                   "internal_rotation": (-30.0, 30.0),
                               },
                               negate_axes={
                                   "flexion": True,
                                   "adduction": False,
                                   "internal_rotation": False,
                               })

        self.left_ankle = Joint(bone=skeleton.left_foot, left_side=True,
                                primary_name="left_ankle", secondary_name="left_foot",
                                axis_names=("dorsiflexion/plantarflexion", "inversion/eversion", "internal/external rotation"),
                                ranges={
                                    "flexion": (-50.0, 30.0),
                                    "adduction": (-30.0, 30.0),
                                    "internal_rotation": (-45.0, 45.0),
                                })

        # Right leg
        self.right_hip = Joint(bone=skeleton.right_upper_leg, left_side=False,
                               primary_name="right_hip", secondary_name="right_upper_leg",
                               ranges={
                                   "flexion": (-30.0, 130.0),
                                   "adduction": (-60.0, 30.0),
                                   "internal_rotation": (-45.0, 45.0),
                               })

        self.right_knee = Joint(bone=skeleton.right_lower_leg, left_side=False,
                                primary_name="right_knee", secondary_name="right_lower_leg",
                                axis_names=("flexion/extension", "varus/valgus", "internal/external rotation"),
                                ranges={
                                    "flexion": (0.0, 140.0),
                                    "adduction": (-10.0, 10.0),
                                    "internal_rotation": (-30.0, 30.0),
                                },
                                negate_axes={
                                    "flexion": True,
                                    "adduction": False,
                                    "internal_rotation": False,
                                })

        self.right_ankle = Joint(bone=skeleton.right_foot, left_side=False,
                                 primary_name="right_ankle", secondary_name="right_foot",
                                 axis_names=("dorsiflexion/plantarflexion", "inversion/eversion", "internal/external rotation"),
                                 ranges={
                                     "flexion": (-50.0, 30.0),
                                     "adduction": (-30.0, 30.0),
                                     "internal_rotation": (-45.0, 45.0),
                                 })
    
    def get_all_joints(self):
        return [
            # Spine / Axial
            self.head,
            self.neck,
            self.lumbar,

            # Left arm
            self.left_shoulder,
            self.left_elbow,
            self.left_wrist,

            # Right arm
            self.right_shoulder,
            self.right_elbow,
            self.right_wrist,

            # Left leg
            self.left_hip,
            self.left_knee,
            self.left_ankle,
            
            # Right leg
            self.right_hip,
            self.right_knee,
            self.right_ankle,
        ]

    def compute_angles(self):
        for joint in self.get_all_joints():
            joint.compute_angles()
    