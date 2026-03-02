"""
Virtual Sensor System

Provides a virtual sensor that can operate independently or bridge to a real sensor.
Allows sensor operations (offset, tare, etc.) to work without physical hardware.
"""

from typing import Optional, List, Tuple, Any
from yostlabs.math.quaternion import quat_mul, quat_inverse, quaternion_swap_axes
from yostlabs.tss3.api import ThreespaceCmdResult, ThreespaceHeader, ThreespaceSensor
import time

class VirtualSensor:
    """
    Virtual sensor that maintains all sensor state and can perform operations
    without requiring a physical sensor connection.
    """
    
    def __init__(self):
        # Orientation state (quaternions: [x, y, z, w])
        self.offset_quat: List[float] = [0, 0, 0, 1]
        self.base_offset_quat: List[float] = [0, 0, 0, 1]
        self.tare_quat: List[float] = [0, 0, 0, 1]
        self.base_tare_quat: List[float] = [0, 0, 0, 1]
        self.untared_orientation: List[float] = [0, 0, 0, 1]
        
        # Component data
        self.accel: List[float] = [0.0, 0.0, 0.0]
        self.gyro: List[float] = [0.0, 0.0, 0.0]
        self.mag: List[float] = [0.0, 0.0, 0.0]
        
        # Configuration
        self.axis_order: str = "xyz"
        self.tare_auto_base: bool = True
        self.axis_offset_enabled: bool = True
        
        # Streaming state
        self.stream_hz: int = 33
        self.last_output_time: float = 0.0
    
    # ==================================================================
    # ORIENTATION OPERATIONS
    # ==================================================================
    
    def setOffsetWithCurrentOrientation(self):
        """
        Set offset quaternion based on current untared orientation.
        If tare_auto_base is enabled, also updates base_tare.
        """
        self.offset_quat = quat_inverse(quat_mul(self.base_offset_quat, self.untared_orientation))
        if self.tare_auto_base:
            self.base_tare_quat = self.offset_quat
        
    
    def setBaseOffsetWithCurrentOrientation(self):
        """
        Set base offset quaternion based on current untared orientation.
        """
        self.base_offset_quat = quat_inverse(self.untared_orientation)
    
    def tareWithCurrentOrientation(self):
        """
        Set tare quaternion based on current untared orientation.
        """
        self.tare_quat = quat_inverse(quat_mul(self.untared_orientation, self.base_tare_quat))
    
    def setBaseTareWithCurrentOrientation(self):
        """
        Set base tare quaternion based on current untared orientation.
        
        TODO: Implement logic to:
        - Calculate base tare from current untared orientation
        - Update self.base_tare_quat
        """
        self.base_tare_quat = quat_inverse(self.untared_orientation)
    
    # ==================================================================
    # SETTINGS MANAGEMENT
    # ==================================================================
    
    def set_settings(self, **kwargs):
        """
        Set multiple sensor settings at once.
        
        Supported settings:
        - offset: List[float] - Offset quaternion
        - base_offset: List[float] - Base offset quaternion
        - tare_quat: List[float] - Tare quaternion
        - base_tare: List[float] - Base tare quaternion
        - axis_order: str - Axis order string (e.g., "xyz", "-x-yz")
        - tare_auto_base: int/bool - Auto base tare on offset
        - axis_offset_enabled: int/bool - Apply axis offset to component data
        - stream_hz: int - Streaming frequency
        - stream_slots: List[Any] - Streaming data slots
        
        TODO: Implement logic to:
        - Parse and validate kwargs
        - Update corresponding attributes
        - Handle type conversions (int to bool, etc.)
        """
        for key, value in kwargs.items():
            if key == "offset":
                self.offset_quat = list(value)
            elif key == "base_offset":
                self.base_offset_quat = list(value)
            elif key == "tare_quat":
                self.tare_quat = list(value)
            elif key == "base_tare":
                self.base_tare_quat = list(value)
            elif key == "axis_order":
                #Convert the untared quaternion to the new space. On a real sensor this would happen automatically
                #sense the sensors actual orientation would not change. But have to do manually here for virtual sensor.
                self.untared_orientation = quaternion_swap_axes(self.untared_orientation, self.axis_order, str(value))
                self.axis_order = str(value)
            elif key == "tare_auto_base":
                self.tare_auto_base = bool(int(value)) if isinstance(value, (int, str)) else bool(value)
            elif key == "axis_offset_enabled":
                self.axis_offset_enabled = bool(int(value)) if isinstance(value, (int, str)) else bool(value)
            elif key == "stream_hz":
                self.stream_hz = int(value)
    
    def __get_axis_order_compass_notation(self) -> str:
        """
        Convert current axis order to compass notation (N/S E/W U/D).
        
        Returns:
            Axis order string in compass notation.
        """
        order = self.axis_order.lower()
        compass_map = {'x': 'E', 'y': 'U', 'z': 'N'}
        neg_compass_map = {'x': 'W', 'y': 'D', 'z': 'S'}

        result = []
        
        i = 0
        while i < len(order):
            map = compass_map
            if order[i] == '-':
                map = neg_compass_map
                i += 1
            result.append(map[order[i]])
            i += 1
        return ''.join(result)

    def get_settings(self, settings_str: str) -> Any:
        """
        Get sensor settings by name.
        
        Args:
            settings_str: Semicolon-separated list of settings to retrieve
                         (e.g., "offset;tare_quat" or single "axis_order_c")
        
        Returns:
            If single setting: string value
            If multiple settings: dict mapping setting name to string value
        
        TODO: Implement logic to:
        - Parse settings_str
        - Look up requested settings
        - Format values as strings (e.g., quaternions as "x,y,z,w")
        - Return dict for multiple settings, string for single setting
        - Handle axis_order_c (compass notation conversion)
        """
        settings = [s.strip() for s in settings_str.split(';')]
        
        result = {}
        for setting in settings:
            if setting == "offset":
                result[setting] = ','.join(str(v) for v in self.offset_quat)
            elif setting == "base_offset":
                result[setting] = ','.join(str(v) for v in self.base_offset_quat)
            elif setting == "tare_quat":
                result[setting] = ','.join(str(v) for v in self.tare_quat)
            elif setting == "base_tare":
                result[setting] = ','.join(str(v) for v in self.base_tare_quat)
            elif setting == "axis_order":
                result[setting] = self.axis_order
            elif setting == "axis_order_c":
                result[setting] = self.__get_axis_order_compass_notation()
            elif setting == "tare_auto_base":
                result[setting] = "1" if self.tare_auto_base else "0"
            elif setting == "axis_offset_enabled":
                result[setting] = "1" if self.axis_offset_enabled else "0"
        
        # Return single value if only one setting requested, otherwise dict
        if len(settings) == 1:
            return result.get(settings[0], "")
        return result
    
    # ==================================================================
    # STREAMING OPERATIONS
    # ==================================================================
    
    def startStreaming(self):
        """
        Start streaming sensor data.
        """
        #Trigger instant output
        self.last_output_time = time.perf_counter() - (1 / self.stream_hz)
    
    def stopStreaming(self):
        """
        Stop streaming sensor data.
        """
        pass
    
    def updateStreaming(self):
        """
        Update streaming data (called each frame).
        """
        pass
    
    def getNewestStreamingPacket(self) -> ThreespaceCmdResult[list[Any]]:
        """
        Get the most recent streaming data packet.
        
        Returns:
            Packet object with .data attribute containing streamed values,
            or None if no data available.
        NOTE: This is hard coded to match the expected output for this application
        """
        #The timer is just so this simulates outputting at the set rate to prevent
        #anything relying on the timing from this to run more often then expected (such as updates/draws)
        cur_time = time.perf_counter()
        if cur_time - self.last_output_time < (1 / self.stream_hz):
            return None # Not time for new packet yet

        #Build Packet as expected by using application. Not configurable, specific
        #to the application.
        data = [self.untared_orientation, self.accel, self.gyro, self.mag]
        return ThreespaceCmdResult(data, ThreespaceHeader())
    
    def clearStreamingPackets(self):
        """
        Clear accumulated streaming packets.
        """
        pass
    
    # ==================================================================
    # LIFECYCLE
    # ==================================================================
    
    def cleanup(self):
        """
        Clean up sensor resources.
        
        TODO: Implement cleanup logic if needed
        """
        pass


class SensorBridge(VirtualSensor):
    """
    Bridges a virtual sensor to a real physical sensor.
    Allows operations to be performed virtually and synced to hardware,
    or to use real sensor data while maintaining virtual state.
    """
    
    def __init__(self, real_sensor: Optional[ThreespaceSensor] = None, use_real_data: bool = True):
        """
        Initialize sensor bridge.
        
        Args:
            real_sensor: Optional physical sensor object to bridge to
            use_real_data: If True, use real sensor's data; if False, use virtual data
        """
        super().__init__()
        self.real_sensor: Optional[ThreespaceSensor] = real_sensor
        self.use_real_data: bool = use_real_data
    
    def attach_sensor(self, sensor: Any):
        """
        Attach a real sensor to the bridge.
        
        Args:
            sensor: Physical sensor object to attach
        """
        self.real_sensor = sensor
    
    def detach_sensor(self):
        """
        Detach the real sensor, reverting to pure virtual mode.
        """
        self.real_sensor = None
    
    # ==================================================================
    # ORIENTATION OPERATIONS (with sensor sync)
    # ==================================================================
    
    def setOffsetWithCurrentOrientation(self):
        """
        Set offset quaternion based on current orientation.
        Syncs to real sensor if available.
        """
        if self.use_real_data and self.real_sensor is not None:
            # Use real sensor's operation
            self.real_sensor.setOffsetWithCurrentOrientation()
            # Sync back to virtual state
            result = self.real_sensor.get_settings("offset;base_tare;tare_auto_base")
            self.offset_quat = [float(v) for v in result["offset"].split(',')]
            if result["tare_auto_base"] == "1":
                self.base_tare_quat = [float(v) for v in result["base_tare"].split(',')]
        else:
            # Use virtual operation
            super().setOffsetWithCurrentOrientation()
            # Push to real sensor if available
            if self.real_sensor is not None:
                self.real_sensor.set_settings(offset=self.offset_quat)
                if self.tare_auto_base:
                    self.real_sensor.set_settings(base_tare=self.base_tare_quat)
    
    def setBaseOffsetWithCurrentOrientation(self):
        """
        Set base offset quaternion. Syncs to real sensor if available.
        """
        if self.use_real_data and self.real_sensor is not None:
            self.real_sensor.setBaseOffsetWithCurrentOrientation()
            result = self.real_sensor.get_settings("base_offset")
            self.base_offset_quat = [float(v) for v in result.split(',')]
        else:
            super().setBaseOffsetWithCurrentOrientation()
            if self.real_sensor is not None:
                self.real_sensor.set_settings(base_offset=self.base_offset_quat)
    
    def tareWithCurrentOrientation(self):
        """
        Set tare quaternion. Syncs to real sensor if available.
        """
        if self.use_real_data and self.real_sensor is not None:
            self.real_sensor.tareWithCurrentOrientation()
            result = self.real_sensor.get_settings("tare_quat")
            self.tare_quat = [float(v) for v in result.split(',')]
        else:
            super().tareWithCurrentOrientation()
            if self.real_sensor is not None:
                self.real_sensor.set_settings(tare_quat=self.tare_quat)
    
    def setBaseTareWithCurrentOrientation(self):
        """
        Set base tare quaternion. Syncs to real sensor if available.
        """
        if self.use_real_data and self.real_sensor is not None:
            self.real_sensor.setBaseTareWithCurrentOrientation()
            result = self.real_sensor.get_settings("base_tare")
            self.base_tare_quat = [float(v) for v in result.split(',')]
        else:
            super().setBaseTareWithCurrentOrientation()
            if self.real_sensor is not None:
                self.real_sensor.set_settings(base_tare=self.base_tare_quat)
    
    # ==================================================================
    # SETTINGS MANAGEMENT (with sensor sync)
    # ==================================================================
    
    def set_settings(self, **kwargs):
        """
        Set settings on virtual sensor and sync to real sensor if available.
        """
        super().set_settings(**kwargs)
        if self.real_sensor is not None:
            self.real_sensor.set_settings(**kwargs)
    
    def get_settings(self, settings_str: str) -> Any:
        """
        Get settings, preferring real sensor if use_real_data is True.
        """
        if self.use_real_data and self.real_sensor is not None:
            result = self.real_sensor.get_settings(settings_str)
            # TODO: Parse result and update virtual state
            return result
        else:
            return super().get_settings(settings_str)
    
    # ==================================================================
    # STREAMING OPERATIONS (with sensor sync)
    # ==================================================================
    
    def startStreaming(self):
        """
        Start streaming on real sensor if available, otherwise virtual.
        """
        super().startStreaming()
        if self.real_sensor is not None:
            self.real_sensor.startStreaming()
    
    def stopStreaming(self):
        """
        Stop streaming on real sensor if available.
        """
        super().stopStreaming()
        if self.real_sensor is not None:
            self.real_sensor.stopStreaming()
    
    def updateStreaming(self):
        """
        Update streaming from real sensor if available.
        
        TODO: Implement logic to:
        - If real_sensor exists, call its updateStreaming()
        - Update virtual state with streamed data if needed
        """
        super().updateStreaming()
        if self.real_sensor is not None:
            self.real_sensor.updateStreaming()
    
    def getNewestStreamingPacket(self):
        """
        Get streaming packet from real sensor if available.
        
        TODO: Implement logic to:
        - If real_sensor exists, return its packet
        - Otherwise return virtual packet (or None)
        """
        if self.real_sensor is not None and self.use_real_data:
            packet = self.real_sensor.getNewestStreamingPacket()
            if packet is not None: #Required expected value from main. Change this if the data is changed.
                self.untared_orientation = packet.data[0]
                self.accel = packet.data[1]
                self.gyro = packet.data[2]
                self.mag = packet.data[3]
            return packet
        return super().getNewestStreamingPacket()
    
    def clearStreamingPackets(self):
        """
        Clear packets from real sensor if available.
        
        TODO: Implement cleanup
        """
        if self.real_sensor is not None:
            self.real_sensor.clearStreamingPackets()
    
    # ==================================================================
    # LIFECYCLE
    # ==================================================================
    
    def cleanup(self):
        """
        Clean up both virtual and real sensor resources.
        
        TODO: Implement cleanup logic for both virtual and real sensor
        """
        if self.real_sensor is not None:
            self.real_sensor.cleanup()
        super().cleanup()
