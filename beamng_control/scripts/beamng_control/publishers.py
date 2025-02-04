from abc import ABC, abstractmethod
import rospy
from rospy import Time
import numpy as np

np.float = np.float64  # temp fix for following import
import ros_numpy
import tf
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from typing import Dict, List, Any, Type, Tuple
import threading

import geometry_msgs.msg as geom_msgs
from geometry_msgs.msg import Point as geom_msgs_Point
from geometry_msgs.msg import PoseStamped, TransformStamped  # Correct import for ROS1

from visualization_msgs.msg import Marker, MarkerArray
import tf.transformations as tf_transformations
from tf.transformations import quaternion_from_euler, quaternion_multiply
from tf import transformations as tf_trans 
import beamng_msgs.msg as bng_msgs
import beamngpy.sensors as bng_sensors
import std_msgs.msg
from sensor_msgs.msg import Range, Imu, NavSatFix, NavSatStatus, Image
from sensor_msgs.point_cloud2 import PointCloud2
from sensor_msgs.msg import PointField  # Add this import



try:
    import radar_msgs.msg as radar_msgs
    RADAR_MSGS_FOUND = True
except ImportError as e:
    RADAR_MSGS_FOUND = False


def get_sensor_publisher(sensor):
    """
    Identifies and returns the appropriate publisher for a given sensor type.
    
    Args:
        sensor: The sensor instance to be matched with a publisher class.
    
    Returns:
        A class that corresponds to the given sensor instance.
    """
    sensor_mapping = {
        bng_sensors.State: StatePublisher,
        bng_sensors.Timer: TimerPublisher,
        bng_sensors.Damage: DamagePublisher,
        bng_sensors.GForces: GForcePublisher,
        bng_sensors.Electrics: ElectricsPublisher,
        bng_sensors.IdealRadar: IdealRadarPublisher,
        bng_sensors.Radar: RadarPublisher,
        bng_sensors.RoadsSensor: RoadsSensorPublisher,
        bng_sensors.AdvancedIMU: AdvancedIMUPublisher,
        bng_sensors.Mesh: MeshPublisher,
        bng_sensors.PowertrainSensor: PowertrainSensorPublisher,
        bng_sensors.GPS: GPSPublisher
    }
    for k, v in sensor_mapping.items():
        if isinstance(sensor, k):
            return v
    rospy.logerr(f'Could not identify publisher for {type(sensor)}')


class BNGPublisher(ABC):
    """
    Abstract base class for BeamNG publishers.

    The publisher classes that inherit from this should implement the `publish` method
    to define how the data will be published to the appropriate ROS topics.
    """

    @abstractmethod
    def publish(self, current_time):
        """
        Abstract method for publishing data to a ROS topic.
        
        Args:
            current_time: The current time for publishing, typically ROS time.
        """
        pass


class SensorDataPublisher(BNGPublisher):
    """
    Base class for sensor data publishers.

    Args:
        sensor: The sensor instance to poll data from.
        topic_id: The ROS topic on which to publish the data.
        msg_type: The message type that corresponds to the sensor.
    """
    def __init__(self, sensor, topic_id, msg_type):
        rospy.logdebug(f'Publishing to {topic_id}')
        self._sensor = sensor
        self._pub = rospy.Publisher(topic_id, msg_type, queue_size=1)
        self.current_time = rospy.get_rostime()
        self.frame_map = 'map'

    @abstractmethod
    def _make_msg(self):
        """
        Abstract method for creating a ROS message from sensor data.
        Must be implemented by subclasses based on the specific sensor data format.
        """
        pass

    def publish(self, current_time):
        """
        Publish the message created by `_make_msg` to the ROS topic.

        Args:
            current_time: The current time for publishing.
        """
        self.current_time = current_time
        msg = self._make_msg()
        self._pub.publish(msg)


class RadarPublisher(SensorDataPublisher):
    """
    Publishes radar sensor data as `RadarScan` messages.

    Args:
        sensor: The radar sensor to be polled.
        topic_id: The ROS topic for publishing radar data.
        vehicle: The vehicle object that the radar is attached to.
    """

    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, bng_msgs.RadarScan)
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_Radar_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()

    def _convert_array_to_dict(self, data_array):
        """
        Convert the radar sensor's NumPy data array to a dictionary for easier access.

        Args:
            data_array: NumPy array containing radar sensor data.
        
        Returns:
            Dictionary mapping radar data fields to their values.
        """
        return {
            "range": data_array[0],
            "doppler_velocity": data_array[1],
            "azimuth": data_array[2],
            "elevation": data_array[3],
            "radar_cross_section": data_array[4],
            "signal_to_noise_ratio": data_array[5],
            "facing_factor": data_array[6],
        }

    def _make_msg(self):
        """
        Converts radar sensor data into a ROS `RadarScan` message.
        
        Returns:
            A ROS message of type `RadarScan` if data is available and valid, otherwise None.
        """
        data = self._sensor.poll()
        if data is None:
            rospy.logwarn("No data received from Radar sensor.")
            return None

        if isinstance(data, np.ndarray):
            data = self._convert_array_to_dict(data)

        if not isinstance(data, dict):
            rospy.logwarn(f"Unexpected data type received: {type(data)}")
            return None

        try:
            range_msg = float(data.get("range", 0.0)[0])
            doppler_velocity_msg = float(data.get("doppler_velocity", 0.0)[0])
            azimuth_msg = float(data.get("azimuth", 0.0)[0])
            elevation_msg = float(data.get("elevation", 0.0)[0])
            radar_cross_section_msg = float(data.get("radar_cross_section", 0.0)[0])
            signal_to_noise_ratio_msg = float(data.get("signal_to_noise_ratio", 0.0)[0])
            facing_factor_msg = float(data.get("facing_factor", 0.0)[0])
        except Exception as e:
            rospy.logwarn(f"Error extracting data fields: {str(e)}")
            return None

        radar_return_msg = bng_msgs.RadarReturn(
            range=range_msg,
            doppler_velocity=doppler_velocity_msg,
            azimuth=azimuth_msg,
            elevation=elevation_msg,
            radar_cross_section=radar_cross_section_msg,
            signal_to_noise_ratio=signal_to_noise_ratio_msg,
            facing_factor=facing_factor_msg,
        )

        header = std_msgs.msg.Header(
            stamp=self.current_time,
            frame_id=self.frame_Radar_sensor
        )

        msg = bng_msgs.RadarScan(
            header=header,
            returns=[radar_return_msg]
        )

        return msg

class RoadsSensorPublisher(SensorDataPublisher):
    """
    Publishes roads sensor data as `RoadsSensor` messages.

    Args:
        sensor: The roads sensor to be polled.
        topic_id: The ROS topic for publishing roads data.
        vehicle: The vehicle object that the sensor is attached to.
    """

    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, bng_msgs.RoadsSensor)
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_Roads_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()

    @staticmethod
    def _make_cubic_polynomial(a: float, b: float, c: float, d: float) -> bng_msgs.CubicPolynomial:
        """
        Helper method to create a cubic polynomial message.

        Args:
            a: Coefficient a of the polynomial.
            b: Coefficient b of the polynomial.
            c: Coefficient c of the polynomial.
            d: Coefficient d of the polynomial.
        
        Returns:
            A `CubicPolynomial` message.
        """
        return bng_msgs.CubicPolynomial(a=a, b=b, c=c, d=d)

    def xyz_to_point(self, x: float, y: float, z: float) -> geom_msgs.Point:
        """
        Convert 3D coordinates into a ROS `Point`.

        Args:
            x: X-coordinate.
            y: Y-coordinate.
            z: Z-coordinate.
        
        Returns:
            A `Point` message with the given coordinates.
        """
        return geom_msgs.Point(x=x, y=y, z=z)

    def _make_msg(self):
        """
        Converts roads sensor data into a ROS `RoadsSensor` message.
        
        Returns:
            A ROS message of type `RoadsSensor` if data is available and valid, otherwise None.
        """
        data = self._sensor.poll()

        if not data:
            rospy.logwarn("No data received from RoadsSensor.")
            return None

        if not isinstance(data, dict):
            rospy.logwarn(f"Unexpected data type received: {type(data)}")
            return None

        valid_data = {}
        for key, value in data.items():
            if isinstance(value, dict):
                valid_data[key] = value
            else:
                rospy.logwarn(f"Invalid data entry for key {key}: Expected dict, got {type(value)}")

        if not valid_data:
            rospy.logwarn("No valid data entries found in RoadsSensor data.")
            return None

        first_key = next(iter(valid_data))
        data = valid_data[first_key]

        # Extract and convert road sensor data fields into a message
        dist2_cl = data.get("dist2CL", 0.0)
        dist2_left = data.get("dist2Left", 0.0)
        dist2_right = data.get("dist2Right", 0.0)
        half_width = data.get("halfWidth", 0.0)
        road_radius = data.get("roadRadius", 0.0)
        heading_angle = data.get("headingAngle", 0.0)

        p0_on_cl = self.xyz_to_point(data.get("xP0onCL", 0.0), data.get("yP0onCL", 0.0), data.get("zP0onCL", 0.0))
        p1_on_cl = self.xyz_to_point(data.get("xP1onCL", 0.0), data.get("yP1onCL", 0.0), data.get("zP1onCL", 0.0))
        p2_on_cl = self.xyz_to_point(data.get("xP2onCL", 0.0), data.get("yP2onCL", 0.0), data.get("zP2onCL", 0.0))
        p3_on_cl = self.xyz_to_point(data.get("xP3onCL", 0.0), data.get("yP3onCL", 0.0), data.get("zP3onCL", 0.0))

        u_cl = self._make_cubic_polynomial(data.get("uAofCL", 0.0), data.get("uBofCL", 0.0), data.get("uCofCL", 0.0), data.get("uDofCL", 0.0))
        v_cl = self._make_cubic_polynomial(data.get("vAofCL", 0.0), data.get("vBofCL", 0.0), data.get("vCofCL", 0.0), data.get("vDofCL", 0.0))

        u_left_re = self._make_cubic_polynomial(data.get("uAofLeftRE", 0.0), data.get("uBofLeftRE", 0.0), data.get("uCofLeftRE", 0.0), data.get("uDofLeftRE", 0.0))
        v_left_re = self._make_cubic_polynomial(data.get("vAofLeftRE", 0.0), data.get("vBofLeftRE", 0.0), data.get("vCofLeftRE", 0.0), data.get("vDofLeftRE", 0.0))

        u_right_re = self._make_cubic_polynomial(data.get("uAofRightRE", 0.0), data.get("uBofRightRE", 0.0), data.get("uCofRightRE", 0.0), data.get("uDofRightRE", 0.0))
        v_right_re = self._make_cubic_polynomial(data.get("vAofRightRE", 0.0), data.get("vBofRightRE", 0.0), data.get("vCofRightRE", 0.0), data.get("vDofRightRE", 0.0))

        start_cl = self.xyz_to_point(data.get("xStartCL", 0.0), data.get("yStartCL", 0.0), data.get("zStartCL", 0.0))
        start_l = self.xyz_to_point(data.get("xStartL", 0.0), data.get("yStartL", 0.0), data.get("zStartL", 0.0))
        start_r = self.xyz_to_point(data.get("xStartR", 0.0), data.get("yStartR", 0.0), data.get("zStartR", 0.0))

        drivability = data.get("drivability", 0.0)
        speed_limit = data.get("speedLimit", 0.0)
        flag1way = data.get("flag1way", 0.0)

        msg = bng_msgs.RoadsSensor(
            header=std_msgs.msg.Header(
                stamp=self.current_time,
                frame_id=self.frame_Roads_sensor
            ),
            dist2_cl=dist2_cl,
            dist2_left=dist2_left,
            dist2_right=dist2_right,
            half_width=half_width,
            road_radius=road_radius,
            heading_angle=heading_angle,
            p0_on_cl=p0_on_cl,
            p1_on_cl=p1_on_cl,
            p2_on_cl=p2_on_cl,
            p3_on_cl=p3_on_cl,
            u_cl=u_cl,
            v_cl=v_cl,
            u_left_re=u_left_re,
            v_left_re=v_left_re,
            u_right_re=u_right_re,
            v_right_re=v_right_re,
            start_cl=start_cl,
            start_l=start_l,
            start_r=start_r,
            drivability=drivability,
            speed_limit=speed_limit,
            flag1way=flag1way
        )
        return msg


class IdealRadarPublisher(SensorDataPublisher):
    """
    Publishes ideal radar sensor data as `IdealRadarSensor` messages.

    Args:
        sensor: The radar sensor to be polled.
        topic_id: The ROS topic for publishing radar data.
        vehicle: The vehicle object that the sensor is attached to.
    """

    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, bng_msgs.IdealRadarSensor)
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_IdealRadar_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()

    def xyz_to_vec3(self, x: float, y: float, z: float) -> geom_msgs.Vector3:
        """
        Converts x, y, z coordinates into a ROS `Vector3` message.

        Args:
            x: X-coordinate.
            y: Y-coordinate.
            z: Z-coordinate.
        
        Returns:
            A `Vector3` message.
        """
        return geom_msgs.Vector3(x=x, y=y, z=z)

    def _vehicle_to_msg(self, veh: Dict[str, Any]) -> bng_msgs.IdealRadarSensorVehicle:
        """
        Converts a vehicle's information dictionary into an `IdealRadarSensorVehicle` message.

        Args:
            veh: Dictionary containing vehicle data.
        
        Returns:
            An `IdealRadarSensorVehicle` message.
        """
        vel = self.xyz_to_vec3(**veh.get("vel", {"x": 0, "y": 0, "z": 0}))

        return bng_msgs.IdealRadarSensorVehicle(
            vehicle_id=int(veh["vehicleID"]),
            dist_to_player_vehicle_sq=veh["distToPlayerVehicleSq"],
            width=veh["width"],
            length=veh["length"],
            acc=self.xyz_to_vec3(**veh.get("acc", {"x": 0, "y": 0, "z": 0})),
            vel=vel,
            rel_acc_x=veh.get("relAccX", 0),
            rel_acc_y=veh.get("relAccY", 0),
            rel_dist_x=veh.get("relDistX", 0),
            rel_dist_y=veh.get("relDistY", 0),
            rel_vel_x=veh.get("relVelX", 0),
            rel_vel_y=veh.get("relVelY", 0),
        )

    def _make_msg(self):
        """
        Converts the ideal radar sensor data into a ROS `IdealRadarSensor` message.

        Returns:
            A ROS `IdealRadarSensor` message.
        """
        data = self._sensor.poll()
        if 0.0 in data:  # bulk data
            data = data[0.0]

        msg = bng_msgs.IdealRadarSensor(
            header=std_msgs.msg.Header(
                stamp=self.current_time,
                frame_id=self.frame_IdealRadar_sensor
            ),
            closest_vehicles1=self._vehicle_to_msg(data["closestVehicles1"]),
            closest_vehicles2=self._vehicle_to_msg(data["closestVehicles2"]),
            closest_vehicles3=self._vehicle_to_msg(data["closestVehicles3"]),
            closest_vehicles4=self._vehicle_to_msg(data["closestVehicles4"]),
        )
        return msg


class PowertrainSensorPublisher(SensorDataPublisher):
    """
    Publishes powertrain sensor data as `PowertrainSensor` messages.

    Args:
        sensor: The powertrain sensor to be polled.
        topic_id: The ROS topic for publishing powertrain data.
        vehicle: The vehicle object that the sensor is attached to.
    """

    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, bng_msgs.PowertrainSensor)
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_Powertrain_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()

    @staticmethod
    def _device_to_msg(name: str, device: Dict[str, Any]) -> bng_msgs.PowertrainSensorDevice:
        """
        Converts powertrain device data into a `PowertrainSensorDevice` message.

        Args:
            name: The name of the powertrain device.
            device: Dictionary containing the device's data.

        Returns:
            A `PowertrainSensorDevice` message.
        """
        if isinstance(device, dict):
            return bng_msgs.PowertrainSensorDevice(
                name=name,
                input_av=device["inputAV"],
                gear_ratio=device.get("gearRatio", float("nan")),
                is_broken=device.get("isBroken", False),
                mode=device.get("mode", ""),
                parent_name=device.get("parentName", ""),
                parent_output_index=int(device.get("parentOutputIndex", -1)),
                output_torque_1=device.get("outputTorque1", float("nan")),
                output_av_1=device.get("outputAV1", float("nan")),
                output_torque_2=device.get("outputTorque2", float("nan")),
                output_av_2=device.get("outputAV2", float("nan")),
            )
        else:
            rospy.logwarn(f"Expected dict for device, got {type(device)}: {device}")
            return bng_msgs.PowertrainSensorDevice(
                name=name,
                input_av=float("nan"),
                gear_ratio=float("nan"),
                is_broken=False,
                mode="",
                parent_name="",
                parent_output_index=-1,
                output_torque_1=float("nan"),
                output_av_1=float("nan"),
                output_torque_2=float("nan"),
                output_av_2=float("nan"),
            )

    def _make_msg(self):
        """
        Converts the powertrain sensor data into a ROS `PowertrainSensor` message.

        Returns:
            A ROS `PowertrainSensor` message.
        """
        data = self._sensor.poll()
        if 0.0 in data:  # bulk data
            data = data[0.0]

        msg = bng_msgs.PowertrainSensor(
            header=std_msgs.msg.Header(
                stamp=self.current_time,
                frame_id=self.frame_Powertrain_sensor
            ),
            devices=[
                self._device_to_msg(name, device) for name, device in data.items()
            ],
        )
        return msg


class MeshPublisher(SensorDataPublisher):
    """
    Publishes mesh sensor data as `MeshSensor` messages.

    Args:
        sensor: The mesh sensor to be polled.
        topic_id: The ROS topic for publishing mesh data.
        vehicle: The vehicle object that the sensor is attached to.
    """

    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, bng_msgs.MeshSensor)
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_Mesh_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()

    def xyz_to_point(self, x: float, y: float, z: float) -> geom_msgs.Point:
        """
        Converts x, y, z coordinates into a ROS `Point` message.

        Args:
            x: X-coordinate.
            y: Y-coordinate.
            z: Z-coordinate.

        Returns:
            A `Point` message.
        """
        return geom_msgs.Point(x=x, y=y, z=z)

    def xyz_to_vec3(self, x: float, y: float, z: float) -> geom_msgs.Vector3:
        """
        Converts x, y, z coordinates into a ROS `Vector3` message.

        Args:
            x: X-coordinate.
            y: Y-coordinate.
            z: Z-coordinate.

        Returns:
            A `Vector3` message.
        """
        return geom_msgs.Vector3(x=x, y=y, z=z)

    @staticmethod
    def _beam_to_msg(beam: Dict[str, Any]) -> bng_msgs.MeshSensorBeam:
        """
        Converts beam data into a `MeshSensorBeam` message.

        Args:
            beam: Dictionary containing beam data.

        Returns:
            A `MeshSensorBeam` message.
        """
        return bng_msgs.MeshSensorBeam(stress=beam["stress"])

    def _node_to_msg(self, node: Dict[str, Any]) -> bng_msgs.MeshSensorNode:
        """
        Converts node data into a `MeshSensorNode` message.

        Args:
            node: Dictionary containing node data.

        Returns:
            A `MeshSensorNode` message.
        """
        return bng_msgs.MeshSensorNode(
            part_origin=node.get("partOrigin", ""),
            mass=node["mass"],
            pos=self.xyz_to_point(**node["pos"]),
            vel=self.xyz_to_vec3(**node["vel"]),
            force=self.xyz_to_vec3(**node["force"]),
        )

    def _make_msg(self):
        """
        Converts mesh sensor data into a ROS `MeshSensor` message.

        Returns:
            A ROS `MeshSensor` message.
        """
        data = self._sensor.poll()
        msg = bng_msgs.MeshSensor(
            header=std_msgs.msg.Header(
                stamp=self.current_time,
                frame_id=self.frame_Mesh_sensor
            ),
            beams=[self._beam_to_msg(data["beams"][i]) for i in range(len(data["beams"]))],
            nodes=[self._node_to_msg(data["nodes"][i]) for i in range(len(data["nodes"]))],
        )
        return msg


class GPSPublisher(SensorDataPublisher):
    """
    Publishes GPS sensor data as `NavSatFix` messages.

    Args:
        sensor: The GPS sensor to be polled.
        topic_id: The ROS topic for publishing GPS data.
        vehicle: The vehicle object that the sensor is attached to.
    """

    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, NavSatFix)
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_Gps_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()

    def _make_msg(self):
        """
        Converts GPS sensor data into a ROS `NavSatFix` message.

        Returns:
            A ROS `NavSatFix` message.
        """
        data = self._sensor.poll()
        if 0.0 in data:
            data = data[0.0]

        msg = NavSatFix(
            header=std_msgs.msg.Header(
                stamp=self.current_time,
                frame_id=self.frame_Gps_sensor
            ),
            status=NavSatStatus(
                status=NavSatStatus.STATUS_FIX,
                service=NavSatStatus.SERVICE_GPS
            ),
            latitude=data["lat"],
            longitude=data["lon"],
            position_covariance_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN,
        )
        return msg


class StatePublisher(SensorDataPublisher):
    """
    Publishes vehicle state data as `StateSensor` messages.

    Args:
        sensor: The state sensor to be polled.
        topic_id: The ROS topic for publishing state data.
    """

    def __init__(self, sensor, topic_id):
        super().__init__(sensor, topic_id, bng_msgs.StateSensor)

    def _make_msg(self):
        """
        Converts the vehicle state sensor data into a ROS `StateSensor` message.

        Returns:
            A ROS `StateSensor` message.
        """
        data = self._sensor.data
        msg = bng_msgs.StateSensor()
        msg.position = data['pos']
        msg.velocity = data['vel']
        msg.front = data['front']
        msg.up = data['up']
        msg.dir = data['dir']
        return msg


class TimerPublisher(SensorDataPublisher):
    """
    Publishes time sensor data as `TimeSensor` messages.

    Args:
        sensor: The timer sensor to be polled.
        topic_id: The ROS topic for publishing time data.
    """

    def __init__(self, sensor, topic_id):
        super().__init__(sensor, topic_id, bng_msgs.TimeSensor)

    def _make_msg(self):
        """
        Converts the time sensor data into a ROS `TimeSensor` message.

        Returns:
            A ROS `TimeSensor` message.
        """
        msg = bng_msgs.TimeSensor()
        data = self._sensor.data
        seconds = int(data['time'])
        msg.beamng_simulation_time.data.set(int(seconds), 0)
        return msg


class DamagePublisher(SensorDataPublisher):
    """
    Publishes damage sensor data as `DamageSensor` messages.

    Args:
        sensor: The damage sensor to be polled.
        topic_id: The ROS topic for publishing damage data.
    """

    def __init__(self, sensor, topic_id):
        super().__init__(sensor, topic_id, bng_msgs.DamageSensor)

    def _make_msg(self):
        """
        Converts the damage sensor data into a ROS `DamageSensor` message.

        Returns:
            A ROS `DamageSensor` message.
        """
        msg = bng_msgs.DamageSensor()
        data = self._sensor.data
        for k, v in data['deform_group_damage'].items():
            msg.deform_group.deformgroup_id.append(k)
            msg.deform_group.invMaxEvents.append(v['invMaxEvents'])
            msg.deform_group.damage.append(v['damage'])
            msg.deform_group.eventCount.append(v['eventCount'])
            msg.deform_group.maxEvents.append(v['maxEvents'])
        if data['part_damage']:
            for k, v in data['part_damage'].items():
                msg.part_damage.part_id.append(k)
                msg.part_damage.name.append(v['name'])
                msg.part_damage.damage.append(v['damage'])
        return msg


class GForcePublisher(SensorDataPublisher):
    """
    Publishes g-force sensor data as `GForceSensor` messages.

    Args:
        sensor: The g-force sensor to be polled.
        topic_id: The ROS topic for publishing g-force data.
    """

    def __init__(self, sensor, topic_id):
        super().__init__(sensor, topic_id, bng_msgs.GForceSensor)

    def _make_msg(self):
        """
        Converts the g-force sensor data into a ROS `GForceSensor` message.

        Returns:
            A ROS `GForceSensor` message.
        """
        data = self._sensor.data
        msg = bng_msgs.GForceSensor()
        msg.gx = data['gx']
        msg.gy = data['gy']
        msg.gz = data['gz']
        msg.gx2 = data['gx2']
        msg.gy2 = data['gy2']
        msg.gz2 = data['gz2']
        return msg


class ElectricsPublisher(SensorDataPublisher):
    """
    Publishes electrics sensor data as `ElectricsSensor` messages.

    Args:
        sensor: The electrics sensor to be polled.
        topic_id: The ROS topic for publishing electrics data.
    """

    def __init__(self, sensor, topic_id):
        super().__init__(sensor, topic_id, bng_msgs.ElectricsSensor)

    def _make_msg(self):
        """
        Converts the electrics sensor data into a ROS `ElectricsSensor` message.

        Returns:
            A ROS `ElectricsSensor` message.
        """
        data = self._sensor.data
        msg = bng_msgs.ElectricsSensor()
        msg.accXSmooth = data['accXSmooth']
        msg.accYSmooth = data['accYSmooth']
        msg.accZSmooth = data['accZSmooth']
        msg.isABSBrakeActive = data['isABSBrakeActive']
        msg.trip = data['trip']
        msg.airflowspeed = data['airflowspeed']
        msg.parkingbrake_input = data['parkingbrake_input']
        msg.parkingbrakelight = data['parkingbrakelight']
        msg.parking = data['parking']
        msg.hazard = data['hazard']
        msg.hoodCatchCoupler_notAttached = data['hoodCatchCoupler_notAttached']
        msg.oil = data['oil']
        msg.lowhighbeam = data['lowhighbeam']
        msg.lowbeam = data['lowbeam']
        msg.highbeam = data['highbeam']
        msg.lowhighbeam_signal_R = data['lowhighbeam_signal_R']
        msg.lowhighbeam_signal_L = data['lowhighbeam_signal_L']
        msg.gear = data['gear']
        msg.brake_input = data['brake_input']
        msg.hasABS = data['hasABS']
        msg.throttle = data['throttle']
        msg.doorFLCoupler_notAttached = data['doorFLCoupler_notAttached']
        msg.highbeam_wigwag_L = data['highbeam_wigwag_L']
        msg.reverse_wigwag_R = data['reverse_wigwag_R']
        msg.reverse_wigwag_L = data['reverse_wigwag_L']
        msg.reverse = data['reverse']
        msg.turboBoost = data['turboBoost']
        msg.turboBoostMax = data['turboBoostMax']
        msg.brakelight_signal_R = data['brakelight_signal_R']
        msg.brakelight_signal_L = data['brakelight_signal_L']
        msg.ignitionLevel = data['ignitionLevel']
        msg.abs = data['abs']
        msg.minGearIndex = data['minGearIndex']
        msg.maxGearIndex = data['maxGearIndex']
        msg.nop = data['nop']
        msg.gearboxMode = data['gearboxMode']
        msg.gearModeIndex = data['gearModeIndex']
        msg.virtualAirspeed = data['virtualAirspeed']
        msg.driveshaft = data['driveshaft']
        msg.steering = data['steering']
        msg.lockupClutchRatio = data['lockupClutchRatio']
        msg.turboRpmRatio = data['turboRpmRatio']
        msg.turboSpin = data['turboSpin']
        msg.turboRPM = data['turboRPM']
        msg.smoothShiftLogicAV = data['smoothShiftLogicAV']
        msg.steering_input = data['steering_input']
        msg.engineRunning = data['engineRunning']
        msg.rpmspin = data['rpmspin']
        msg.boost = data['boost']
        msg.boostMax = data['boostMax']
        msg.idlerpm = data['idlerpm']
        msg.lights = data['lights']
        msg.turnsignal = data['turnsignal']
        msg.altitude = data['altitude']
        msg.odometer = data['odometer']
        msg.rpm = data['rpm']
        msg.lightbar = data['lightbar']
        msg.maxrpm = data['maxrpm']
        msg.tailgateCoupler_notAttached = data['tailgateCoupler_notAttached']
        msg.clutch = data['clutch']
        msg.fuel = data['fuel']
        msg.hasTCS = data['hasTCS']
        msg.doorRRCoupler_notAttached = data['doorRRCoupler_notAttached']
        msg.airspeed = data['airspeed']
        msg.tcs = data['tcs']
        msg.doorRLCoupler_notAttached = data['doorRLCoupler_notAttached']
        msg.esc = data['esc']
        msg.dseWarningPulse = data['dseWarningPulse']
        msg.highbeam_wigwag_R = data['highbeam_wigwag_R']
        msg.parkingbrake = data['parkingbrake']
        msg.isYCBrakeActive = data['isYCBrakeActive']
        msg.doorFRCoupler_notAttached = data['doorFRCoupler_notAttached']
        msg.steeringUnassisted = data['steeringUnassisted']
        msg.clutch_input = data['clutch_input']
        msg.throttle_input = data['throttle_input']
        msg.horn = data['horn']
        msg.brake = data['brake']
        msg.wheelspeed = data['wheelspeed']
        msg.hoodLatchCoupler_notAttached = data['hoodLatchCoupler_notAttached']
        msg.lowpressure = data['lowpressure']
        msg.isTCBrakeActive = data['isTCBrakeActive']
        msg.hasESC = data['hasESC']
        msg.abs_active = data['abs_active']
        msg.avg_wheel_av = data['avg_wheel_av']
        msg.brake_lights = data['brake_lights']
        msg.clutch_ratio = data['clutch_ratio']
        msg.engine_load = data['engine_load']
        msg.engine_throttle = data['engine_throttle']
        msg.exhaust_flow = data['exhaust_flow']
        msg.fog_lights = data['fog_lights']
        msg.fuel_volume = data['fuel_volume']
        msg.fuel_capacity = data['fuel_capacity']
        msg.gear_a = data['gear_a']
        msg.gear_index = data['gear_index']
        msg.gear_m = data['gear_m']
        msg.headlights = data['headlights']
        msg.oil_temperature = data['oil_temperature']
        msg.radiator_fan_spin = data['radiator_fan_spin']
        msg.rpm_tacho = data['rpm_tacho']
        msg.signal_l = data['signal_l']
        msg.signal_r = data['signal_r']
        msg.water_temperature = data['water_temperature']
        msg.two_step = data['two_step']
        msg.running = data['running']
        msg.ignition = data['ignition']
        msg.freezeState = data['freezeState']
        msg.left_signal = data['left_signal']
        msg.hazard_signal = data['hazard_signal']
        msg.esc_active = data['esc_active']
        msg.check_engine = data['check_engine']
        msg.lowfuel = data['lowfuel']
        msg.right_signal = data['right_signal']
        msg.tcs_active = data['tcs_active']
        msg.is_shifting = data['is_shifting']
        return msg


class CameraDataPublisher:
    """
    Base class for publishing camera data to ROS.

    Args:
        sensor: The camera sensor object.
        topic_id: The ROS topic to publish the camera data.
        msg_type: The ROS message type for the camera data.
    """

    def __init__(self, sensor, topic_id, msg_type):
        rospy.logdebug(f'Publishing to {topic_id}')
        self._sensor = sensor
        self._pub = rospy.Publisher(topic_id, msg_type, queue_size=1)
        self.current_time = rospy.get_rostime()
        self.frame_map = 'map'

    @abstractmethod
    def _make_msg(self, data):
        """
        Abstract method for creating a ROS message from the camera data.

        Args:
            data: The data to convert into a ROS message.
        """
        pass

    def publish(self, current_time, data):
        """
        Publish the message created by `_make_msg` to the ROS topic.

        Args:
            current_time: The current time for publishing.
            data: The data to be published.
        """
        self.current_time = current_time
        msg = self._make_msg(data)
        self._pub.publish(msg)


class ColorImgPublisher(CameraDataPublisher):
    """
    Publishes color image data as `Image` messages.

    Args:
        sensor: The camera sensor to be polled.
        topic_id: The ROS topic for publishing color image data.
        cv_helper: The CvBridge helper for converting OpenCV images to ROS Image messages.
        data_descriptor: The key to access image data in the sensor data.
    """

    def __init__(self, sensor, topic_id, cv_helper, data_descriptor):
        super().__init__(sensor, topic_id, Image)
        self._cv_helper = cv_helper
        self._data_descriptor = data_descriptor

    def _make_msg(self, data):
        """
        Converts the color image sensor data into a ROS `Image` message.

        Args:
            data: The sensor data containing the color image.
        
        Returns:
            A ROS `Image` message.
        """
        img = data[self._data_descriptor]
        if img is not None:
            img = np.array(img.convert('RGB'))
            img = img[:, :, ::-1].copy()  # Convert to BGR format for ROS compatibility
        else:
            img = np.zeros_like(data['colour'].convert('RGB'))
        try:
            img = self._cv_helper.cv2_to_imgmsg(img, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
        return img


class DepthImgPublisher(CameraDataPublisher):
    """
    Publishes depth image data as `Image` messages.

    Args:
        sensor: The depth camera sensor to be polled.
        topic_id: The ROS topic for publishing depth image data.
        cv_helper: The CvBridge helper for converting OpenCV images to ROS Image messages.
    """

    def __init__(self, sensor, topic_id, cv_helper):
        super().__init__(sensor, topic_id, Image)
        self._cv_helper = cv_helper

    def _make_msg(self, data):
        """
        Converts the depth image sensor data into a ROS `Image` message.

        Args:
            data: The sensor data containing the depth image.
        
        Returns:
            A ROS `Image` message.
        """
        img = data['depth']
        near, far = self._sensor.near_far_planes
        img = (np.array(img) - near) / far * 255
        img = img.astype(np.uint8)
        try:
            img = self._cv_helper.cv2_to_imgmsg(img, 'mono8')
        except CvBridgeError as e:
            rospy.logerr(e)
        return img


class BBoxImgPublisher(CameraDataPublisher):
    """
    Publishes bounding box image data as `Image` messages.

    Args:
        sensor: The camera sensor to be polled.
        topic_id: The ROS topic for publishing bounding box image data.
        cv_helper: The CvBridge helper for converting OpenCV images to ROS Image messages.
        vehicle: The vehicle object associated with the bounding boxes.
    """

    def __init__(self, sensor, topic_id, cv_helper, vehicle):
        super().__init__(sensor, topic_id, Image)
        self._cv_helper = cv_helper
        self._vehicle = vehicle
        self._classes = None

    def _update_data_with_bbox(self, data):
        """
        Adds bounding boxes to the image data.

        Args:
            data: The sensor data containing the image and annotation information.
        
        Returns:
            The image with bounding boxes drawn.
        """
        if self._classes is None:
            annotations = self._vehicle.bng.get_annotations()
            self._classes = self._vehicle.bng.get_annotation_classes(annotations)
        bboxes = bng_sensors.Camera.extract_bboxes(data['annotation'], data['instance'], self._classes)
        bboxes = [b for b in bboxes if b['class'] == 'CAR']
        rospy.logdebug(f'bboxes: {bboxes}')
        bbox_img = bng_sensors.Camera.draw_bboxes(bboxes, data['colour'], width=3)
        return bbox_img

    def _make_msg(self, data):
        """
        Converts the bounding box image sensor data into a ROS `Image` message.

        Args:
            data: The sensor data containing the image and bounding boxes.
        
        Returns:
            A ROS `Image` message.
        """
        img = self._update_data_with_bbox(data)
        img = img.convert('RGB')
        img = np.array(img)
        img = img[:, :, ::-1].copy()
        try:
            img = self._cv_helper.cv2_to_imgmsg(img, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
        return img


class CameraPublisher(BNGPublisher):
    """
    Camera data publisher for multiple types of camera sensor data (color, depth, annotations).

    Args:
        sensor: The camera sensor object.
        topic_id: The ROS topic base for publishing camera data.
        vehicle: The vehicle object the camera is attached to.
    """

    def __init__(self, sensor, topic_id, vehicle):
        self._sensor = sensor
        self._cv_helper = CvBridge()
        self._publishers = list()
        if self._sensor.is_render_colours:
            color_topic = '/'.join([topic_id, 'colour'])
            pub = ColorImgPublisher(sensor, color_topic, self._cv_helper, 'colour')
            self._publishers.append(pub)
        if self._sensor.is_render_depth:
            depth_topic = '/'.join([topic_id, 'depth'])
            pub = DepthImgPublisher(sensor, depth_topic, self._cv_helper)
            self._publishers.append(pub)
        if self._sensor.is_render_annotations:
            annotation_topic = '/'.join([topic_id, 'annotation'])
            pub = ColorImgPublisher(sensor, annotation_topic, self._cv_helper, 'annotation')
            self._publishers.append(pub)
        if self._sensor.is_render_instance:
            inst_ann_topic = '/'.join([topic_id, 'instance'])
            pub = ColorImgPublisher(sensor, inst_ann_topic, self._cv_helper, 'instance')
            self._publishers.append(pub)

    def publish(self, current_time):
        """
        Polls the camera sensor data and publishes it using the appropriate publisher(s).

        Args:
            current_time: The current time for publishing.
        """
        if self._sensor.is_render_instance:
            data = self._sensor.get_full_poll_request()
        else:
            data = self._sensor.poll()
        for pub in self._publishers:
            pub.current_time = current_time
            pub.publish(current_time, data)
            
            
class UltrasonicPublisher(SensorDataPublisher):
    """
    Publishes ultrasonic sensor data as `Range` messages.

    Args:
        sensor: The ultrasonic sensor to be polled.
        topic_id: The ROS topic for publishing ultrasonic sensor data.
        vehicle: The vehicle object that the sensor is attached to.
    """

    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, Range)
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_USSensor_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()

    def _make_msg(self):
        """
        Converts the ultrasonic sensor data into a ROS `Range` message.

        Returns:
            A ROS `Range` message containing the sensor data.
        """
        data = self._sensor.poll()
        USSensor_msg = Range()
        USSensor_msg.radiation_type = Range.ULTRASOUND
        USSensor_msg.header.frame_id = self.frame_USSensor_sensor
        USSensor_msg.header.stamp = self.current_time
        USSensor_msg.field_of_view = 5.7
        USSensor_msg.min_range = 0.1
        USSensor_msg.max_range = 5.0
        USSensor_msg.range = data['distance']

        try:
            (trans_map, _) = self.listener.lookupTransform(self.frame_map, self.frame_USSensor_sensor, USSensor_msg.header.stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f'No transform between {self.frame_map} and {self.frame_USSensor_sensor} available with exception: {e}')

        return USSensor_msg


class AdvancedIMUPublisher(SensorDataPublisher):
    """
    Publishes IMU sensor data as `Imu` messages.

    Args:
        sensor: The IMU sensor to be polled.
        topic_id: The ROS topic for publishing IMU data.
        vehicle: The vehicle object that the sensor is attached to.
    """

    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, Imu)
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_ImuSensor_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()

    def _make_msg(self):
        """
        Converts the IMU sensor data into a ROS `Imu` message.

        Returns:
            A ROS `Imu` message containing the sensor data.
        """
        data = self._sensor.poll()

        if data is None or len(data) < 2:
            rospy.logerr("Invalid sensor data. Skipping IMU message publication.")
            return None

        try:
            imu_msg = Imu()
            imu_msg.header.stamp = self.current_time
            imu_msg.header.frame_id = 'advanced_imu_sensor'

            imu_msg.orientation.x = data[1]['dirX'][0]
            imu_msg.orientation.y = data[1]['dirY'][1]
            imu_msg.orientation.z = data[1]['dirZ'][2]
            imu_msg.orientation.w = 1.0  # Assume the quaternion is normalized

            imu_msg.angular_velocity.x = data[1]['angVel'][0]
            imu_msg.angular_velocity.y = data[1]['angVel'][1]
            imu_msg.angular_velocity.z = data[1]['angVel'][2]

            imu_msg.linear_acceleration.x = data[1]['accRaw'][0]
            imu_msg.linear_acceleration.y = data[1]['accRaw'][1]
            imu_msg.linear_acceleration.z = data[1]['accRaw'][2]

            imu_msg.orientation_covariance = np.zeros(9)
            imu_msg.angular_velocity_covariance = np.zeros(9)
            imu_msg.linear_acceleration_covariance = np.zeros(9)

            return imu_msg

        except KeyError as e:
            rospy.logerr(f"KeyError encountered: {e}")
            return None
        except IndexError as e:
            rospy.logerr(f"IndexError encountered: {e}")
            return None


class LidarPublisher(SensorDataPublisher):
    """
    Publishes lidar sensor data as `PointCloud2` messages.

    Args:
        sensor: The lidar sensor to be polled.
        topic_id: The ROS topic for publishing lidar data.
        vehicle: The vehicle object that the sensor is attached to.
    """

    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, PointCloud2)
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_lidar_sensor = f'{vehicle.vid}_{sensor_name}'



    def _make_msg(self):
        header = std_msgs.msg.Header()
        header.frame_id = self.frame_lidar_sensor
        header.stamp = self.current_time

        readings_data = self._sensor.poll()
        if not isinstance(readings_data, dict):
            rospy.logwarn(f"Unexpected sensor data type: {type(readings_data)} - {readings_data}")
            return  # Skip further processing

        points = np.array(readings_data['pointCloud'])
        colours = np.array(readings_data['colours'])
        
        # Ensure we have valid data
        if points.size == 0:
            rospy.logwarn(f'Empty point cloud for sensor {self.frame_lidar_sensor}')
            return PointCloud2()

        num_points = points.shape[0]
        
        # Ensure colours array matches points array length
        if colours.size > 0:
            # Truncate or pad colours array to match number of points
            if colours.shape[0] > num_points:
                colours = colours[:num_points]
            elif colours.shape[0] < num_points:
                # Pad with zeros if we have fewer colours than points
                pad_length = num_points - colours.shape[0]
                colours = np.pad(colours, ((0, pad_length), (0, 0)), mode='constant')
            intensities = colours[:, 0]
        else:
            intensities = np.zeros(num_points)

        pointcloud_fields = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)]

        # rospy.loginfo(f"Point cloud size: {points.shape[0]}")
        if points.shape[0] < 10:
            rospy.logwarn("Sparse point cloud detected.")

        try:
            (trans_map, rot_map) = self.listener.lookupTransform(self.frame_map, self.frame_lidar_sensor, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Transform missing, falling back to identity.")
            trans_map = np.zeros(3)
            rot_map = tf.transformations.quaternion_from_euler(0, 0, 0)

        rotation_matrix = tf_transformations.quaternion_matrix(rot_map)[:3, :3]
        
        # Avoid broadcasting error
        if points.shape[0] > 0:
            rotated_points = np.dot(points - trans_map, rotation_matrix.T)
        else:
            rotated_points = np.zeros((0, 3))  # Handle empty case gracefully

        # Ensure all arrays have matching sizes
        pointcloud_data = np.zeros(rotated_points.shape[0], dtype=pointcloud_fields)
        pointcloud_data['x'] = rotated_points[:, 0]
        pointcloud_data['y'] = rotated_points[:, 1]
        pointcloud_data['z'] = rotated_points[:, 2]
        pointcloud_data['intensity'] = intensities[:rotated_points.shape[0]]  # Ensure matching size

        msg = ros_numpy.msgify(PointCloud2, pointcloud_data)
        msg.header = header
        return msg




class VehiclePublisher(BNGPublisher):
    """
    Publishes vehicle-related data, including sensor data and vehicle pose,
    to the appropriate ROS topics.

    Args:
        vehicle: The vehicle object containing sensors to be polled.
        node_name: The name of the ROS node.
        visualize: Boolean to indicate whether to publish a visualization marker.
    """

    def __init__(self, vehicle, node_name, visualize=True):
        self._vehicle = vehicle
        self.node_name = node_name
        self._sensor_publishers = list()

        # Set up a broadcaster for vehicle pose
        self._broadcaster_pose = tf2_ros.TransformBroadcaster()
        self.tf_msg = tf2_ros.TransformStamped()
        self.frame_map = 'map'
        self.tf_msg.header.frame_id = self.frame_map
        self.tf_msg.child_frame_id = self._vehicle.vid

        self.current_time = rospy.get_rostime()

        # Set up publishers for each sensor
        for sensor_name, sensor in vehicle.sensors.items():
            topic_id = [node_name, vehicle.vid, sensor_name]
            topic_id = '/'.join([str(x) for x in topic_id])
            pub = get_sensor_publisher(sensor)
            rospy.logdebug(f'pub: {pub}')
            if pub == CameraPublisher:
                pub = pub(sensor, topic_id, self._vehicle)
            else:
                pub = pub(sensor, topic_id)
            self._sensor_publishers.append(pub)

        # Optionally set up visualization for vehicle position
        self.visualizer = None
        if visualize:
            topic_id = [node_name, vehicle.vid, 'marker']
            topic_id = '/'.join(topic_id)
            self.visualizer = rospy.Publisher(topic_id, Marker, queue_size=1)

    def broadcast_vehicle_pose(self, data):
        """
        Broadcasts the vehicle pose as a ROS TF transform.

        Args:
            data: The data containing the vehicle's pose (position and orientation).
        """
        self.tf_msg.header.stamp = self.current_time
        self.tf_msg.transform.translation.x = data['pos'][0]
        self.tf_msg.transform.translation.y = data['pos'][1]
        self.tf_msg.transform.translation.z = data['pos'][2]
        # alignment_quat = np.array([0, 1, 0, 0])  # Sets the forward direction as -y
        # alignment_quat = np.array([1, 0, 0, 0])  # Identity quaternion
        quat_orientation = np.array([data['rotation'][0], data['rotation'][1], data['rotation'][2], data['rotation'][3]])        
        # quat_orientation = quaternion_multiply(alignment_quat, quat_orientation)
        # quat_orientation /= np.linalg.norm(quat_orientation)

        # Adjust orientation for ROS (BeamNG to ROS TF)
        alignment_quat = np.array([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])  # 90° rotation around z-axis
        quat_orientation = quaternion_multiply(alignment_quat, quat_orientation)

        # Normalize the quaternion
        quat_orientation /= np.linalg.norm(quat_orientation)

        self.tf_msg.transform.rotation.x = quat_orientation[0]
        self.tf_msg.transform.rotation.y = quat_orientation[1]
        self.tf_msg.transform.rotation.z = quat_orientation[2]
        self.tf_msg.transform.rotation.w = quat_orientation[3]

        self._broadcaster_pose.sendTransform(self.tf_msg)

    def state_to_marker(self, data, marker_ns):
        """
        Converts the vehicle state into a ROS Marker message for visualization.

        Args:
            data: The data containing the vehicle's state (position, rotation).
            marker_ns: The namespace for the marker.

        Returns:
            A ROS Marker message.
        """
        mark = Marker()
        mark.header.frame_id = self.frame_map
        mark.header.stamp = self.current_time
        mark.type = Marker.SPHERE
        mark.ns = marker_ns
        mark.action = Marker.ADD
        mark.id = 0
        mark.lifetime = rospy.Duration()

        mark.pose.position.x = data['pos'][0]
        mark.pose.position.y = data['pos'][1]
        mark.pose.position.z = data['pos'][2]

        rotation_matrix = tf_trans.quaternion_matrix(data['rotation'])
        rotation_matrix = tf_trans.rotation_matrix(-10.0, [0, 0, 1]) @ rotation_matrix
        new_quaternion = tf_trans.quaternion_from_matrix(rotation_matrix)

        mark.pose.orientation.x = new_quaternion[0]
        mark.pose.orientation.y = new_quaternion[1]
        mark.pose.orientation.z = new_quaternion[2]
        mark.pose.orientation.w = new_quaternion[3]

        mark.scale.x = 5
        mark.scale.y = 1.9
        mark.scale.z = 1.5

        mark.color.r = 0.0
        mark.color.g = 1.0
        mark.color.b = 0.0
        mark.color.a = 0.5

        return mark

    def publish(self, current_time):
        """
        Polls vehicle sensors and publishes data, including vehicle pose and visualization markers.

        Args:
            current_time: The current time for publishing.
        """
        self.current_time = current_time
        self._vehicle.poll_sensors()
        self.broadcast_vehicle_pose(self._vehicle.sensors['state'].data)

        # Start a thread to publish each sensor's data
        for pub in self._sensor_publishers:
            threading.Thread(target=pub.publish, args=(current_time,), daemon=True).start()

        # Publish visualization marker if enabled
        if self.visualizer is not None:
            mark = self.state_to_marker(self._vehicle.sensors['state'].data, self._vehicle.vid)
            self.visualizer.publish(mark)



def beamng_rot_to_ros_coords(quat: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x = 0.7071067811865476  # sqrt(2)/2
    return (
        x * (quat[1] - quat[0]),
        -x * (quat[0] + quat[1]),
        -x * (quat[2] + quat[3]),
        x * (quat[3] - quat[2]),
    )


def beamng_vec_to_ros_coords(vec: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (-vec[1], vec[0], vec[2])


# class VehiclePublisher:
#     def __init__(self, vehicle, node_name, visualize=True):
#         self._vehicle = vehicle
#         self.node_name = node_name
#         self._sensor_publishers = []

#         # Set up a broadcaster for vehicle pose
#         self._broadcaster_pose = tf.TransformBroadcaster()
#         self.tf_msg = TransformStamped()  # Use TransformStamped for TF
#         self.frame_map = 'map'
#         self.tf_msg.header.frame_id = self.frame_map
#         self.tf_msg.child_frame_id = self._vehicle.vid  # Correct attribute

#         # Set up publishers for each sensor
#         for sensor_name, sensor in vehicle.sensors.items():
#             topic_id = [node_name, vehicle.vid, sensor_name]
#             topic_id = '/'.join([str(x) for x in topic_id])
#             pub = get_sensor_publisher(sensor)
#             rospy.logdebug(f'pub: {pub}')
#             if pub == CameraPublisher:
#                 pub = pub(sensor, topic_id, self._vehicle)
#             else:
#                 pub = pub(sensor, topic_id)
#             self._sensor_publishers.append(pub)

#         # Optionally set up visualization for vehicle position
#         self.visualizer = None
#         if visualize:
#             topic_id = [node_name, vehicle.vid, 'marker']
#             topic_id = '/'.join(topic_id)
#             self.visualizer = rospy.Publisher(topic_id, Marker, queue_size=1)

#     def broadcast_vehicle_pose(self, data):
#         """
#         Broadcasts the vehicle pose as a ROS TF transform.

#         Args:
#             data: The data containing the vehicle's pose (position and orientation).
#         """
#         # Transform position and rotation from BeamNG to ROS coordinates
#         ros_position = beamng_vec_to_ros_coords(data['pos'])
#         ros_orientation = beamng_rot_to_ros_coords(data['rotation'])

#         # Populate TransformStamped message
#         self.tf_msg.header.stamp = rospy.Time.now()
#         self.tf_msg.transform.translation.x = ros_position[0]
#         self.tf_msg.transform.translation.y = ros_position[1]
#         self.tf_msg.transform.translation.z = ros_position[2]
#         self.tf_msg.transform.rotation.x = ros_orientation[0]
#         self.tf_msg.transform.rotation.y = ros_orientation[1]
#         self.tf_msg.transform.rotation.z = ros_orientation[2]
#         self.tf_msg.transform.rotation.w = ros_orientation[3]

#         # Broadcast TF
#         self._broadcaster_pose.sendTransform(
#             (
#                 self.tf_msg.transform.translation.x,
#                 self.tf_msg.transform.translation.y,
#                 self.tf_msg.transform.translation.z,
#             ),
#             (
#                 self.tf_msg.transform.rotation.x,
#                 self.tf_msg.transform.rotation.y,
#                 self.tf_msg.transform.rotation.z,
#                 self.tf_msg.transform.rotation.w,
#             ),
#             rospy.Time.now(),
#             self.tf_msg.child_frame_id,
#             self.tf_msg.header.frame_id,
#         )

#     def state_to_marker(self, data, marker_ns):
#         mark = Marker()
#         mark.header.frame_id = self.frame_map
#         mark.header.stamp = rospy.Time.now()
#         mark.type = Marker.SPHERE
#         mark.ns = marker_ns
#         mark.action = Marker.ADD
#         mark.id = 0
#         mark.lifetime = rospy.Duration()

#         ros_position = beamng_vec_to_ros_coords(data['pos'])
#         ros_orientation = beamng_rot_to_ros_coords(data['rotation'])

#         mark.pose.position.x = ros_position[0]
#         mark.pose.position.y = ros_position[1]
#         mark.pose.position.z = ros_position[2]

#         mark.pose.orientation.x = ros_orientation[0]
#         mark.pose.orientation.y = ros_orientation[1]
#         mark.pose.orientation.z = ros_orientation[2]
#         mark.pose.orientation.w = ros_orientation[3]

#         mark.scale.x = 5
#         mark.scale.y = 1.9
#         mark.scale.z = 1.5

#         mark.color.r = 0.0
#         mark.color.g = 1.0
#         mark.color.b = 0.0
#         mark.color.a = 0.5

#         return mark

#     def publish(self, current_time):
#         self._vehicle.poll_sensors()
#         self.broadcast_vehicle_pose(self._vehicle.sensors['state'].data)

#         # Start a thread to publish each sensor's data
#         for pub in self._sensor_publishers:
#             threading.Thread(target=pub.publish, args=(current_time,), daemon=True).start()

#         # Publish visualization marker if enabled
#         if self.visualizer is not None:
#             mark = self.state_to_marker(self._vehicle.sensors['state'].data, self._vehicle.vid)
#             self.visualizer.publish(mark)



class NetworkPublisher(BNGPublisher):
    """
    Publishes road network visualization data as `MarkerArray` messages.

    Args:
        game_client: The game client object to retrieve road data from.
        node_name: The ROS node name.
    """

    def __init__(self, game_client, node_name):
        self.frame_map = 'map'
        self._game_client = game_client
        self._road_network = None
        self._node_name = node_name
        topic_id = '/'.join([node_name, 'road_network'])
        self._pub = rospy.Publisher(topic_id, MarkerArray, queue_size=1)
        self.current_time = rospy.get_rostime()

    def set_up_road_network_viz(self):
        """
        Retrieves and sets up the road network visualization as a `MarkerArray`.
        """
        roads = self._game_client.get_roads()

        # Check if roads data is valid and not empty
        if isinstance(roads, dict) and roads:
            network_def = dict()
            # for r_id, r_inf in roads.items():
            #     if r_inf['drivability'] != '-1':
            #         network_def[int(r_id)] = self._game_client.get_road_edges(r_id)

            for r_id, r_inf in roads.items():
                if r_inf['drivability'] != '-1':
                    try:
                        network_def[int(r_id)] = self._game_client.get_road_edges(r_id)
                    except ValueError:
                        rospy.logwarn(f"Skipping road with non-integer ID: {r_id}")
                        continue


            self._road_network = MarkerArray()
            for r_id, road in network_def.items():
                mark = Marker()
                mark.header = std_msgs.msg.Header()
                mark.header.frame_id = self.frame_map
                mark.header.stamp = self.current_time
                mark.type = Marker.LINE_STRIP
                ns = self._node_name
                mark.ns = ns
                mark.action = Marker.ADD
                mark.id = r_id
                mark.lifetime = rospy.Duration(0)  # Leave them up forever

                mark.pose.position.x = 0
                mark.pose.position.y = 0
                mark.pose.position.z = 0
                mark.pose.orientation.x = 0
                mark.pose.orientation.y = 0
                mark.pose.orientation.z = 0
                mark.pose.orientation.w = 1

                mark.scale.x = 1
                mark.scale.y = 0.5

                mark.color.r = 1
                mark.color.b = 0
                mark.color.g = 0
                mark.color.a = 1
                for r_point in road:
                    r_point = r_point['middle']
                    p = geom_msgs.Point(r_point[0], r_point[1], r_point[2])
                    mark.points.append(p)
                self._road_network.markers.append(mark)
            marker_num = len(self._road_network.markers)
            rospy.logdebug(f'The road network contains {marker_num} markers')
        else:
            rospy.logwarn("No road information available.")

    def publish(self, current_time):
        """
        Publishes the road network visualization to the ROS topic.

        Args:
            current_time: The current time for publishing.
        """
        self.current_time = current_time
        if self._road_network is None:
            self.set_up_road_network_viz()
        self._pub.publish(self._road_network.markers)

class NetworkPublisherM(BNGPublisher):
    """
    Publishes middle lane road network visualization data as `MarkerArray` messages.

    Args:
        game_client: The game client object to retrieve road data from.
        node_name: The ROS node name.
    """

    def __init__(self, game_client, node_name):
        self.frame_map = 'map'
        self._game_client = game_client
        self._road_network = None
        self._node_name = node_name
        topic_id = '/'.join([node_name, 'middle_road_network'])
        self._pub = rospy.Publisher(topic_id, MarkerArray, queue_size=1)
        self.current_time = rospy.get_rostime()

    def set_up_road_network_viz(self):
        """
        Retrieves and sets up the middle lane road network visualization as a `MarkerArray`.
        """
        roads = self._game_client.get_roads()
        network_def = dict()
        for r_id, r_inf in roads.items():
            if r_inf['drivability'] != '-1':
                network_def[int(r_id)] = self._game_client.get_road_edges(r_id)

        self._road_network = MarkerArray()
        for r_id, road in network_def.items():
            mark = Marker()
            mark.header = std_msgs.msg.Header()
            mark.header.frame_id = self.frame_map
            mark.header.stamp = self.current_time
            mark.type = Marker.LINE_STRIP
            mark.ns = self._node_name
            mark.action = Marker.ADD
            mark.id = r_id
            mark.lifetime = rospy.Duration(0)  # Leave them up forever

            mark.pose.position.x = 0
            mark.pose.position.y = 0
            mark.pose.position.z = 0
            mark.pose.orientation.x = 0
            mark.pose.orientation.y = 0
            mark.pose.orientation.z = 0
            mark.pose.orientation.w = 1

            mark.scale.x = 1
            mark.scale.y = 0.5

            mark.color.r = 0.5
            mark.color.b = 0.5
            mark.color.g = 0.5
            mark.color.a = 1

            for r_point in road:
                r_point = r_point['middle']
                p = geom_msgs.Point(r_point[0], r_point[1], r_point[2])
                mark.points.append(p)
            self._road_network.markers.append(mark)

    def publish(self, current_time):
        """
        Publishes the middle lane road network visualization to the ROS topic.

        Args:
            current_time: The current time for publishing.
        """
        self.current_time = current_time
        if self._road_network is None:
            self.set_up_road_network_viz()
        self._pub.publish(self._road_network.markers)


class NetworkPublisherR(BNGPublisher):
    """
    Publishes right lane road network visualization data as `MarkerArray` messages.

    Args:
        game_client: The game client object to retrieve road data from.
        node_name: The ROS node name.
    """

    def __init__(self, game_client, node_name):
        self.frame_map = 'map'
        self._game_client = game_client
        self._road_network = None
        self._node_name = node_name
        topic_id = '/'.join([node_name, 'right_road_network'])
        self._pub = rospy.Publisher(topic_id, MarkerArray, queue_size=1)
        self.current_time = rospy.get_rostime()

    def set_up_road_network_viz(self):
        """
        Retrieves and sets up the right lane road network visualization as a `MarkerArray`.
        """
        roads = self._game_client.get_roads()
        network_def = dict()
        for r_id, r_inf in roads.items():
            if r_inf['drivability'] != '-1':
                network_def[int(r_id)] = self._game_client.get_road_edges(r_id)

        self._road_network = MarkerArray()
        for r_id, road in network_def.items():
            mark = Marker()
            mark.header = std_msgs.msg.Header()
            mark.header.frame_id = self.frame_map
            mark.header.stamp = self.current_time
            mark.type = Marker.LINE_STRIP
            mark.ns = self._node_name
            mark.action = Marker.ADD
            mark.id = r_id
            mark.lifetime = rospy.Duration(0)

            mark.pose.position.x = 0
            mark.pose.position.y = 0
            mark.pose.position.z = 0
            mark.pose.orientation.x = 0
            mark.pose.orientation.y = 0
            mark.pose.orientation.z = 0
            mark.pose.orientation.w = 1

            mark.scale.x = 0.5
            mark.scale.y = 0.5

            mark.color.r = 1
            mark.color.b = 0
            mark.color.g = 0
            mark.color.a = 1

            for r_point in road:
                r_point = r_point['right']
                p = geom_msgs.Point(r_point[0], r_point[1], r_point[2])
                mark.points.append(p)
            self._road_network.markers.append(mark)

    def publish(self, current_time):
        """
        Publishes the right lane road network visualization to the ROS topic.

        Args:
            current_time: The current time for publishing.
        """
        self.current_time = current_time
        if self._road_network is None:
            self.set_up_road_network_viz()
        self._pub.publish(self._road_network.markers)


class NetworkPublisherL(BNGPublisher):
    """
    Publishes left lane road network visualization data as `MarkerArray` messages.

    Args:
        game_client: The game client object to retrieve road data from.
        node_name: The ROS node name.
    """

    def __init__(self, game_client, node_name):
        self.frame_map = 'map'
        self._game_client = game_client
        self._road_network = None
        self._node_name = node_name
        topic_id = '/'.join([node_name, 'left_road_network'])
        self._pub = rospy.Publisher(topic_id, MarkerArray, queue_size=1)
        self.current_time = rospy.get_rostime()

    def set_up_road_network_viz(self):
        """
        Retrieves and sets up the left lane road network visualization as a `MarkerArray`.
        """
        roads = self._game_client.get_roads()
        network_def = dict()
        for r_id, r_inf in roads.items():
            if r_inf['drivability'] != '-1':
                network_def[int(r_id)] = self._game_client.get_road_edges(r_id)

        self._road_network = MarkerArray()
        for r_id, road in network_def.items():
            mark = Marker()
            mark.header = std_msgs.msg.Header()
            mark.header.frame_id = self.frame_map
            mark.header.stamp = self.current_time
            mark.type = Marker.LINE_STRIP
            mark.ns = self._node_name
            mark.action = Marker.ADD
            mark.id = r_id
            mark.lifetime = rospy.Duration(0)

            mark.pose.position.x = 0
            mark.pose.position.y = 0
            mark.pose.position.z = 0
            mark.pose.orientation.x = 0
            mark.pose.orientation.y = 0
            mark.pose.orientation.z = 0
            mark.pose.orientation.w = 1

            mark.scale.x = 0.5
            mark.scale.y = 0.5

            mark.color.r = 1
            mark.color.b = 0
            mark.color.g = 0
            mark.color.a = 1

            for r_point in road:
                r_point = r_point['left']
                p = geom_msgs.Point(r_point[0], r_point[1], r_point[2])
                mark.points.append(p)
            self._road_network.markers.append(mark)

    def publish(self, current_time):
        """
        Publishes the left lane road network visualization to the ROS topic.

        Args:
            current_time: The current time for publishing.
        """
        self.current_time = current_time
        if self._road_network is None:
            self.set_up_road_network_viz()
        self._pub.publish(self._road_network.markers)
