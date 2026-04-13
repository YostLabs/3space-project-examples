from skeleton import Skeleton, Bone, BoneMode
from yostlabs.tss3.api import ThreespaceSensor, StreamableCommands
import yostlabs.math.vector as yl_vec
import yostlabs.math.quaternion as yl_quat
from yostlabs.math.axes import AxisOrder
import math


class ThreespaceBone(Bone):
    """
    Wrapper around a Bone that adds ThreespaceSensor integration.
    
    A ThreespaceBone combines a Bone with an optional ThreespaceSensor,
    allowing automatic updates of bone orientation from sensor data.
    """

    def __init__(self, mode: BoneMode = BoneMode.STATIC_LOCAL, parent: "Bone" = None):
        """
        Create a ThreespaceBone.
        
        Args:
            mode: The bone mode (STATIC_GLOBAL or STATIC_LOCAL)
            parent: The parent bone (None for root bone)
        """
        super().__init__(mode=mode, parent=parent)
        self._sensor: ThreespaceSensor = None

        self.update_rate = 33  # Update rate in Hz (can be adjusted as needed)

        # Optional forward axis for this bone (e.g., to determine facing direction during calibration)
        self._forward_axis = None 
        self.yaw_offset = [0, 0, 0, 1]  # Default yaw offset (identity quaternion)

        self.axis_order = AxisOrder("XYZ") #Default sensor axis order, should be changed to match the skeleton/bone
    
    def set_axis_order(self, axis_order: AxisOrder):
        self.axis_order = axis_order

    def set_forward_axis(self, axis: str):
        """
        Set the forward axis for this bone.
        
        Args:
            axis: The forward axis vector
        """
        multiplier = 1
        if '-' in axis:
            multiplier = -1
            axis = axis.replace('-', '')
        self._forward_axis = yl_vec.axis_to_unit_vector(axis)
        self._forward_axis = [multiplier * v for v in self._forward_axis]

    def get_facing_offset(self):
        """
        Get the yaw offset quaternion to align the bone's forward axis with the sensor's forward direction.
        
        This can be used during calibration to ensure the sensor's orientation matches the skeleton's expected facing direction.
        """
        if self._forward_axis is None or self._sensor is None:
            return None
        
        #For getting the yaw offset, going to use a known axis order
        #instead of having seperate code for each possible order
        order = self._sensor.readAxisOrder()
        order = AxisOrder(order)
        EUN = AxisOrder("EUN")

        facing_vector = yl_quat.quat_rotate_vec(self._sensor.getUntaredOrientation().data, self._forward_axis)
        print(f"{facing_vector=}")
        forward_axis = order.swap_to(EUN, facing_vector)
        print(f"{forward_axis=}")

        #Compute the angle around the up vector
        angle = math.atan2(forward_axis[0], forward_axis[2])

        #Rotations need inverted when swapping handedness
        if EUN.is_right_handed != order.is_right_handed:
            angle = -angle
        
        print(f"Calibrated yaw offset: {math.degrees(angle)} degrees")

        #Create the rotation and swap back to the actual axis space of the sensor
        offset_quat = yl_quat.quat_from_euler([angle], "y")
        offset_quat = EUN.swap_to(order, offset_quat)

        return offset_quat

    def set_yaw_offset(self, offset_quat):
        """
        Sets an offset used when doing calibration for the bone.
        This offset is applied to modify the expected pose.
        
        Args:
            offset_quat: The yaw offset quaternion to apply
        """
        self.yaw_offset = offset_quat

    def calibrate_sensor(self, tare: bool = False):
        """Calibrate the sensor by offsetting it."""
        if self._sensor is None:
            return
        base_offset = yl_quat.quat_mul(self.yaw_offset, self.global_rotation)
        self._sensor.writeBaseOffset(yl_quat.quat_inverse(base_offset))
        self._sensor.setOffsetWithCurrentOrientation()
        if tare:
            self._sensor.writeTareQuat(yl_quat.quat_inverse(self.yaw_offset))
        else:
            self._sensor.writeTareQuat([0, 0, 0, 1])

    def set_sensor(self, sensor: ThreespaceSensor):
        """
        Set the ThreespaceSensor for this bone.
        
        Args:
            sensor: ThreespaceSensor instance to assign to this bone
        """
        self._sensor = sensor
        if sensor is not None:
            self.mode = BoneMode.STATIC_GLOBAL  # Sensor-controlled bones should use global mode
        self.configure_sensor()
    
    def configure_sensor(self):
        if not self._sensor:
            return
        self._sensor.writeAxisOrder(self.axis_order.to_xyz_string())

    def start_updating(self):
        """Start streaming tared orientation from the sensor."""
        if self._sensor is None:
            return
        self._sensor.write_settings(stream_slots=[StreamableCommands.GetTaredOrientation],
                                    stream_hz=self.update_rate,
                                    axis_order=self.axis_order.to_xyz_string())
        self._sensor.startStreaming()  # Stream tared orientation
    
    def stop_updating(self):
        """Stop streaming from the sensor."""
        if self._sensor is None:
            return
        self._sensor.stopStreaming()
    
    def update(self):
        """
        Update the bone's orientation from the sensor.
        
        Retrieves the newest packet and sets the global_rotation
        to the unmodified orientation (no axis swapping yet).
        """
        if self._sensor is None:
            return
        self._sensor.updateStreaming()
        packet = self._sensor.getNewestStreamingPacket()
        if packet:
            # Store the unmodified orientation as global_rotation
            self.global_rotation = packet.data[0]
            self._sensor.clearStreamingPackets()


class ThreespaceSkeleton(Skeleton[ThreespaceBone]):
    """
    Skeleton variant that uses ThreespaceBone instead of Bone.
    
    A ThreespaceSkeleton is identical to Skeleton in structure and behavior,
    but all bones are ThreespaceBone instances, which support sensor integration.
    
    IntelliSense will correctly recognize all bones as ThreespaceBone instances.
    """
    
    def __init__(self, bone_mode: BoneMode = BoneMode.STATIC_LOCAL):
        """
        Create a complete skeleton with all ThreespaceBones.
        
        Args:
            bone_mode: The default mode for all bones (STATIC_GLOBAL or STATIC_LOCAL)
        """
        # Pass ThreespaceBone as the bone_class to parent constructor
        super().__init__(bone_mode=bone_mode, bone_class=ThreespaceBone)
        for bone in self.get_all_bones():
            bone.set_axis_order(self.axis_order)  # Ensure all bones have the skeleton's axis order
    
    @Skeleton.axis_order.setter
    def axis_order(self, value: AxisOrder):
        """Set the axis order of the skeleton, which represents the expected facing direction and up direction of the model."""
        Skeleton.axis_order.fset(self, value)  # Call the parent setter to set the value
        for bone in self.get_all_bones():
            bone.set_axis_order(self.axis_order)  # Ensure all bones have the skeleton's axis order

    def calibrate_sensors(self, rotate_root: bool = True, tare_sensors: bool = False):
        """
        Calibrate all sensors. 
        Call while standing in the same pose as the skeleton model.
        """

        #Used to determine forward direction.
        #User should call bone.set_forward_axis() for at least one bone before 
        #calibrating to ensure correct facing direction.
        offset = None
        bones: list[ThreespaceBone] = self.get_all_bones()
        for bone in bones:
            offset = bone.get_facing_offset()
            if offset is not None:
                break
        
        for bone in bones:
            if offset is not None:
                bone.set_yaw_offset(offset)
            bone.calibrate_sensor(tare_sensors)

            #Bones that don't have sensors will not appear correct relative to the rest of the model
            #unless the bones with sensors are tared to match the expected pose, or the bones without sensors
            #are instead rotated to match the initial facing of the model.
            if rotate_root and not tare_sensors and offset is not None and not bone._sensor:
                bone.global_rotation = yl_quat.quat_mul(offset, bone.global_rotation)

    def start_updating(self):
        """Start streaming on all bones that have sensors assigned."""
        bones: list[ThreespaceBone] = self.get_all_bones()
        for bone in bones:
            bone.start_updating()
    
    def stop_updating(self):
        """Stop streaming on all bones that have sensors assigned."""
        bones: list[ThreespaceBone] = self.get_all_bones()
        for bone in bones:
            bone.stop_updating()
    
    def update(self):
        """Update all bones from their sensors."""
        bones: list[ThreespaceBone] = self.get_all_bones()
        for bone in bones:
            bone.update()
