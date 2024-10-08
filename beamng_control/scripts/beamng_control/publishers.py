
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
from typing import Dict, List, Any, Type
import threading

import geometry_msgs.msg as geom_msgs
from geometry_msgs.msg import Point as geom_msgs_Point
from visualization_msgs.msg import Marker, MarkerArray
import tf.transformations as tf_transformations
from tf.transformations import quaternion_from_euler, quaternion_multiply
from tf import transformations as tf_trans 


import beamng_msgs.msg as bng_msgs
import beamngpy.sensors as bng_sensors
import std_msgs.msg
from sensor_msgs.msg import Range, Imu, NavSatFix, NavSatStatus, Image
from sensor_msgs.point_cloud2 import PointCloud2
try:
    import radar_msgs.msg as radar_msgs

    RADAR_MSGS_FOUND = True
except ImportError as e:
    RADAR_MSGS_FOUND = False

def get_sensor_publisher(sensor):
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
        
        # bng_sensors.imu: IMUPublisher,
    }
    for k, v in sensor_mapping.items():
        if isinstance(sensor, k):
            return v
    rospy.logerr(f'Could not identify publisher for {type(sensor)}')


class BNGPublisher(ABC):

    @abstractmethod
    def publish(self, current_time):
        pass


class SensorDataPublisher(BNGPublisher):

    def __init__(self, sensor, topic_id, msg_type):
        rospy.logdebug(f'publishing to {topic_id}')
        self._sensor = sensor
        self._pub = rospy.Publisher(topic_id,
                                    msg_type,
                                    queue_size=1)
        self.current_time = rospy.get_rostime()
        self.frame_map = 'map'

    @abstractmethod
    def _make_msg(self):
        pass

    def publish(self, current_time):
        self.current_time = current_time
        msg = self._make_msg()
        self._pub.publish(msg)



class RadarPublisher(SensorDataPublisher):
    """
    Radar sensor publisher publishing radar_msgs :radar_msgs:`RadarReturn` messages
    if the library is found, custom :external+beamng_msgs:doc:`interfaces/msg/RadarReturn` messages if not.

    You can force the publisher to always send custom BeamNG message type by setting the
    ``use_beamng_msg_type=True`` argument in the sensor .json definition.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, sensor, topic_id, vehicle):
        """
        Initializes the RadarPublisher class.

        Args:
            sensor: The sensor instance being used to poll radar data.
            topic_id: The ROS topic on which the radar data will be published.
            vehicle: The vehicle object that the sensor is attached to.
        """
        super().__init__(sensor, topic_id, bng_msgs.RadarScan)
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_Radar_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()

    def _convert_array_to_dict(self, data_array):
        """
        Convert the NumPy array from the radar sensor into a dictionary.
        Adjust the indices according to your sensor data structure.
        """
        # Example: Adjust these indices according to the actual data format
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
        data = self._sensor.poll()

        # Ensure data exists
        if data is None:
            rospy.logwarn("No data received from Radar sensor.")
            return None

        # Handle the case where data is a NumPy array
        if isinstance(data, np.ndarray):
            # Convert the ndarray to a dictionary format.
            data = self._convert_array_to_dict(data)
        
        # Check if the polled data is a dictionary
        if not isinstance(data, dict):
            rospy.logwarn(f"Unexpected data type received: {type(data)}")
            return None

        # Log raw data for debugging
        # rospy.loginfo(f"Raw data from Radar sensor: {data}")

        # Extract data fields, ensuring they are floats
        try:
            range_msg = float(data.get("range", 0.0)[0])  # Get the first value
            doppler_velocity_msg = float(data.get("doppler_velocity", 0.0)[0])
            azimuth_msg = float(data.get("azimuth", 0.0)[0])
            elevation_msg = float(data.get("elevation", 0.0)[0])
            radar_cross_section_msg = float(data.get("radar_cross_section", 0.0)[0])
            signal_to_noise_ratio_msg = float(data.get("signal_to_noise_ratio", 0.0)[0])
            facing_factor_msg = float(data.get("facing_factor", 0.0)[0])
        except Exception as e:
            rospy.logwarn(f"Error extracting data fields: {str(e)}")
            return None

        # Create a RadarReturn message
        radar_return_msg = bng_msgs.RadarReturn(
            range=range_msg,
            doppler_velocity=doppler_velocity_msg,
            azimuth=azimuth_msg,
            elevation=elevation_msg,
            radar_cross_section=radar_cross_section_msg,
            signal_to_noise_ratio=signal_to_noise_ratio_msg,
            facing_factor=facing_factor_msg,
        )

        # Ensure the frame ID is correctly set
        header = std_msgs.msg.Header(
            stamp=self.current_time,
            frame_id=self.frame_Radar_sensor  # corrected variable name
        )

        # Build and return the RadarScan message
        msg = bng_msgs.RadarScan(
            header=header,
            returns=[radar_return_msg]  # wrap in a list
        )

        return msg





class RoadsSensorPublisher(SensorDataPublisher):
    """
    Roads sensor publisher publishing :external+beamng_msgs:doc:`interfaces/msg/RoadsSensor` messages.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """
    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, bng_msgs.RoadsSensor)
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_Roads_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()



    @staticmethod
    def _make_cubic_polynomial(
        a: float, b: float, c: float, d: float
    ) -> bng_msgs.CubicPolynomial:
        return bng_msgs.CubicPolynomial(a=a, b=b, c=c, d=d)


    def xyz_to_point(self, x: float, y: float, z: float) -> geom_msgs.Point:
        return geom_msgs.Point(x=x, y=y, z=z)

    def _make_msg(self):
        data = self._sensor.poll()

        # Ensure data exists and is in the expected format
        if not data:
            rospy.logwarn("No data received from RoadsSensor.")
            return None

        # Check if the polled data is a dictionary
        if not isinstance(data, dict):
            rospy.logwarn(f"Unexpected data type received: {type(data)}")
            return None

        # Log raw data for debugging
        # rospy.loginfo(f"Raw data from RoadsSensor: {data}")

        # Iterate through the data dictionary and handle invalid entries (like floats)
        valid_data = {}
        for key, value in data.items():
            if isinstance(value, dict):
                valid_data[key] = value
            else:
                rospy.logwarn(f"Invalid data entry for key {key}: Expected dict, got {type(value)}")
        
        # If no valid data is found, log and return
        if not valid_data:
            rospy.logwarn("No valid data entries found in RoadsSensor data.")
            return None

        # Process the first valid entry (if data is valid)
        first_key = next(iter(valid_data))
        data = valid_data[first_key]

        # Extract data fields
        dist2_cl = data.get("dist2CL", 0.0)
        dist2_left = data.get("dist2Left", 0.0)
        dist2_right = data.get("dist2Right", 0.0)
        half_width = data.get("halfWidth", 0.0)
        road_radius = data.get("roadRadius", 0.0)
        heading_angle = data.get("headingAngle", 0.0)

        p0_on_cl = self.xyz_to_point(
            data.get("xP0onCL", 0.0), data.get("yP0onCL", 0.0), data.get("zP0onCL", 0.0)
        )
        p1_on_cl = self.xyz_to_point(
            data.get("xP1onCL", 0.0), data.get("yP1onCL", 0.0), data.get("zP1onCL", 0.0)
        )
        p2_on_cl = self.xyz_to_point(
            data.get("xP2onCL", 0.0), data.get("yP2onCL", 0.0), data.get("zP2onCL", 0.0)
        )
        p3_on_cl = self.xyz_to_point(
            data.get("xP3onCL", 0.0), data.get("yP3onCL", 0.0), data.get("zP3onCL", 0.0)
        )

        u_cl = self._make_cubic_polynomial(
            data.get("uAofCL", 0.0), data.get("uBofCL", 0.0), data.get("uCofCL", 0.0), data.get("uDofCL", 0.0)
        )
        v_cl = self._make_cubic_polynomial(
            data.get("vAofCL", 0.0), data.get("vBofCL", 0.0), data.get("vCofCL", 0.0), data.get("vDofCL", 0.0)
        )

        u_left_re = self._make_cubic_polynomial(
            data.get("uAofLeftRE", 0.0), data.get("uBofLeftRE", 0.0), data.get("uCofLeftRE", 0.0), data.get("uDofLeftRE", 0.0)
        )
        v_left_re = self._make_cubic_polynomial(
            data.get("vAofLeftRE", 0.0), data.get("vBofLeftRE", 0.0), data.get("vCofLeftRE", 0.0), data.get("vDofLeftRE", 0.0)
        )

        u_right_re = self._make_cubic_polynomial(
            data.get("uAofRightRE", 0.0), data.get("uBofRightRE", 0.0), data.get("uCofRightRE", 0.0), data.get("uDofRightRE", 0.0)
        )
        v_right_re = self._make_cubic_polynomial(
            data.get("vAofRightRE", 0.0), data.get("vBofRightRE", 0.0), data.get("vCofRightRE", 0.0), data.get("vDofRightRE", 0.0)
        )

        start_cl = self.xyz_to_point(
            data.get("xStartCL", 0.0), data.get("yStartCL", 0.0), data.get("zStartCL", 0.0)
        )
        start_l = self.xyz_to_point(
            data.get("xStartL", 0.0), data.get("yStartL", 0.0), data.get("zStartL", 0.0)
        )
        start_r = self.xyz_to_point(
            data.get("xStartR", 0.0), data.get("yStartR", 0.0), data.get("zStartR", 0.0)
        )

        drivability = data.get("drivability", 0.0)
        speed_limit = data.get("speedLimit", 0.0)
        flag1way = data.get("flag1way", 0.0)

        # Log critical fields
        # rospy.loginfo(f"dist2_cl: {dist2_cl}, dist2_left: {dist2_left}, dist2_right: {dist2_right}")
        # rospy.loginfo(f"half_width: {half_width}, road_radius: {road_radius}, heading_angle: {heading_angle}")

        # Build and return the message
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
    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, bng_msgs.IdealRadarSensor)
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_IdealRadar_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()

    def xyz_to_vec3(self, x: float, y: float, z: float) -> geom_msgs.Vector3:
        return geom_msgs.Vector3(x=x, y=y, z=z)

    def _vehicle_to_msg(self, veh: Dict[str, Any]) -> bng_msgs.IdealRadarSensorVehicle:
        # Use default value (Vector3 with zero velocity) if 'vel' key is missing
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
    Powertrain sensor publisher publishing :external+beamng_msgs:doc:`interfaces/msg/PowertrainSensor` messages.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, bng_msgs.PowertrainSensor)
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_Powertrain_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()


    @staticmethod
    def _device_to_msg(
        name: str, device: Dict[str, Any]) -> bng_msgs.PowertrainSensorDevice:
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
    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, bng_msgs.MeshSensor)  # Correct message type
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_Mesh_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()
    
    def xyz_to_point(self, x: float, y: float, z: float) -> geom_msgs.Point:
        return geom_msgs.Point(x=x, y=y, z=z)

    def xyz_to_vec3(self, x: float, y: float, z: float) -> geom_msgs.Vector3:
        return geom_msgs.Vector3(x=x, y=y, z=z)
        
    @staticmethod
    def _beam_to_msg(beam: Dict[str, Any]) -> bng_msgs.MeshSensorBeam:
        return bng_msgs.MeshSensorBeam(stress=beam["stress"])

    def _node_to_msg(self, node: Dict[str, Any]) -> bng_msgs.MeshSensorNode:
        return bng_msgs.MeshSensorNode(
            part_origin=node.get("partOrigin", ""),
            mass=node["mass"],
            pos=self.xyz_to_point(**node["pos"]),
            vel=self.xyz_to_vec3(**node["vel"]),
            force=self.xyz_to_vec3(**node["force"]),
        )

    def _make_msg(self):
        data = self._sensor.poll()
        msg = bng_msgs.MeshSensor(
            header=std_msgs.msg.Header(
                stamp=self.current_time,  # Correct header with timestamp
                frame_id=self.frame_Mesh_sensor
            ),
            beams=[
                self._beam_to_msg(data["beams"][i])
                for i in range(len(data["beams"]))
            ],
            nodes=[
                self._node_to_msg(data["nodes"][i])
                for i in range(len(data["nodes"]))
            ],
        )
        return msg

class GPSPublisher(SensorDataPublisher):
    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, NavSatFix)  
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_Gps_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()

    def _make_msg(self):
        data = self._sensor.poll()
        if 0.0 in data:
            data = data[0.0]
        # Ensure the message type matches what the publisher expects
        msg = NavSatFix(
            header=std_msgs.msg.Header(
                stamp=self.current_time,  # Correct header with timestamp
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

    def __init__(self, sensor, topic_id):
        super().__init__(sensor,
                         topic_id,
                         bng_msgs.StateSensor)

    def _make_msg(self):
        data = self._sensor.data
        msg = bng_msgs.StateSensor()
        msg.position = data['pos']
        msg.velocity = data['vel']
        msg.front = data['front']
        msg.up = data['up']
        msg.dir = data['dir']
        return msg


class TimerPublisher(SensorDataPublisher):

    def __init__(self, sensor, topic_id):
        super().__init__(sensor,
                         topic_id,
                         bng_msgs.TimeSensor)

    def _make_msg(self):
        msg = bng_msgs.TimeSensor()
        data = self._sensor.data
        seconds = int(data['time'])
        # nseconds = (seconds - seconds//1) * 1e9
        msg.beamng_simulation_time.data.set(int(seconds), 0)
        return msg


class DamagePublisher(SensorDataPublisher):

    def __init__(self, sensor, topic_id):
        super().__init__(sensor,
                         topic_id,
                         bng_msgs.DamageSensor)

    def _make_msg(self):
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
                msg.part_damage.name.append(v['name'])  # todo what is the diff to
                msg.part_damage.damage.append(v['damage'])  # todo is it in %?
        return msg


class GForcePublisher(SensorDataPublisher):

    def __init__(self, sensor, topic_id):
        super().__init__(sensor,
                         topic_id,
                         bng_msgs.GForceSensor)

    def _make_msg(self):
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

    def __init__(self, sensor, topic_id):
        super().__init__(sensor,
                         topic_id,
                         bng_msgs.ElectricsSensor)

    def _make_msg(self):
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

    def __init__(self, sensor, topic_id, msg_type):
        rospy.logdebug(f'publishing to {topic_id}')
        self._sensor = sensor
        self._pub = rospy.Publisher(topic_id,
                                    msg_type,
                                    queue_size=1)
        self.current_time = rospy.get_rostime()
        self.frame_map = 'map'

    @abstractmethod
    def _make_msg(self, data):
        pass

    def publish(self, current_time, data):
        self.current_time = current_time
        msg = self._make_msg(data)
        self._pub.publish(msg)


class ColorImgPublisher(CameraDataPublisher):

    def __init__(self, sensor, topic_id, cv_helper, data_descriptor):
        super().__init__(sensor,
                         topic_id,
                         Image)
        self._cv_helper = cv_helper
        self._data_descriptor = data_descriptor

    def _make_msg(self, data):
        img = data[self._data_descriptor]
        if img is not None:
            img = np.array(img.convert('RGB'))
            img = img[:, :, ::-1].copy()
        else:
            img = np.zeros_like(data['colour'].convert('RGB'))
        try:
            img = self._cv_helper.cv2_to_imgmsg(img, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
        return img


class DepthImgPublisher(CameraDataPublisher):

    def __init__(self, sensor, topic_id, cv_helper):
        super().__init__(sensor,
                         topic_id,
                         Image)
        self._cv_helper = cv_helper

    def _make_msg(self, data):
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

    def __init__(self, sensor, topic_id, cv_helper, vehicle):
        super().__init__(sensor,
                         topic_id,
                         Image)
        self._cv_helper = cv_helper
        self._vehicle = vehicle
        self._classes = None

    def _update_data_with_bbox(self, data):
        if self._classes is None:
            annotations = self._vehicle.bng.get_annotations()
            self._classes = self._vehicle.bng.get_annotation_classes(annotations)
        bboxes = bng_sensors.Camera.extract_bboxes(data['annotation'],
                                                   data['instance'],
                                                   self._classes)
        bboxes = [b for b in bboxes if b['class'] == 'CAR']
        rospy.logdebug(f'bboxes: {bboxes}')
        bbox_img = bng_sensors.Camera.draw_bboxes(bboxes,
                                                  data['colour'],
                                                  width=3)
        return bbox_img

    def _make_msg(self, data):
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
    def __init__(self, sensor, topic_id, vehicle):
        self._sensor = sensor
        self._cv_helper = CvBridge()
        self._publishers = list()
        if self._sensor.is_render_colours:
            color_topic = '/'.join([topic_id, 'colour'])
            pub = ColorImgPublisher(sensor,
                                    color_topic,
                                    self._cv_helper,
                                    'colour')
            self._publishers.append(pub)
        if self._sensor.is_render_depth:
            depth_topic = '/'.join([topic_id, 'depth'])
            pub = DepthImgPublisher(sensor,
                                    depth_topic,
                                    self._cv_helper)
            self._publishers.append(pub)
        if self._sensor.is_render_annotations:
            annotation_topic = '/'.join([topic_id, 'annotation'])
            pub = ColorImgPublisher(sensor,
                                    annotation_topic,
                                    self._cv_helper,
                                    'annotation')
            self._publishers.append(pub)
        if self._sensor.is_render_instance:
            inst_ann_topic = '/'.join([topic_id, 'instance'])
            pub = ColorImgPublisher(sensor,
                                    inst_ann_topic,
                                    self._cv_helper,
                                    'instance')
            self._publishers.append(pub)
        # if self._sensor.bbox:
        #     bbox_topic = '/'.join([topic_id, 'bounding_box'])
        #     pub = BBoxImgPublisher(sensor,
        #                            bbox_topic,
        #                            self._cv_helper,
        #                            vehicle)
        #     self._publishers.append(pub)

    def publish(self, current_time):
        if self._sensor.is_render_instance:
            data = self._sensor.get_full_poll_request()
        else:
            data = self._sensor.poll()
        for pub in self._publishers:
            pub.current_time = current_time
            pub.publish(current_time, data)

class UltrasonicPublisher(SensorDataPublisher):
    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, Range)
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_USSensor_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()

    def _make_msg(self):      
        data = self._sensor.poll()
        USSensor_msg = Range()
        USSensor_msg.radiation_type = Range.ULTRASOUND
        # USSensor_msg.radiation_type = Range.INFRARED
        USSensor_msg.header.frame_id =  self.frame_USSensor_sensor 
        USSensor_msg.header.stamp = self.current_time
        # USSensor_msg.field_of_view = 0.1
        USSensor_msg.field_of_view = 5.7
        USSensor_msg.min_range = 0.1
        USSensor_msg.max_range = 5.0
        # USSensor_msg.max_range = 9999.900390625
        USSensor_msg.range =  data['distance']


        try:
            (trans_map, _) = self.listener.lookupTransform(self.frame_map, self.frame_USSensor_sensor, USSensor_msg.header.stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f'No transform between {self.frame_map} and '
                          f'{self.frame_USSensor_sensor} available with exception: {e}')
            

        return USSensor_msg


class AdvancedIMUPublisher(SensorDataPublisher):
    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, Imu)
        self.listener = tf.TransformListener()
        sensor_name = topic_id.split("/")[-1]
        self.frame_ImuSensor_sensor = f'{vehicle.vid}_{sensor_name}'
        self.current_time = rospy.get_rostime()


    def _make_msg(self):
        data = self._sensor.poll()

        # Debugging to check the returned data
        # rospy.logdebug(f"Sensor data: {data}")

        # Ensure data is not None and is in the expected format
        if data is None or len(data) < 2:
            rospy.logerr("Invalid sensor data. Skipping IMU message publication.")
            return None

        try:
            imu_msg = Imu()
            imu_msg.header.stamp = self.current_time
            imu_msg.header.frame_id = 'advanced_imu_sensor'  # Can change to desired frame id

            # Orientation quaternion (make sure it's a full quaternion [x, y, z, w])
            imu_msg.orientation.x = data[1]['dirX'][0]
            imu_msg.orientation.y = data[1]['dirY'][1]
            imu_msg.orientation.z = data[1]['dirZ'][2]
            imu_msg.orientation.w = 1.0  # Set `w` component (assumed to be normalized)

            # Angular velocity
            imu_msg.angular_velocity.x = data[1]['angVel'][0]
            imu_msg.angular_velocity.y = data[1]['angVel'][1]
            imu_msg.angular_velocity.z = data[1]['angVel'][2]

            # Linear acceleration
            imu_msg.linear_acceleration.x = data[1]['accRaw'][0]
            imu_msg.linear_acceleration.y = data[1]['accRaw'][1]
            imu_msg.linear_acceleration.z = data[1]['accRaw'][2]

            # Optionally, you can also assign covariance matrices if applicable
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
        points = np.array(readings_data['pointCloud'])
        colours = np.array(readings_data['colours'])
        num_points = points.shape[0]
        colours = colours[:num_points]

        # Extract intensity
        intensities = colours[:, 0] if colours.size > 0 else np.zeros((num_points,))

        pointcloud_fields = [('x', np.float32),
                            ('y', np.float32),
                            ('z', np.float32),
                            ('intensity', np.float32)]

        try:
            (trans_map, rot_map) = self.listener.lookupTransform(self.frame_map, self.frame_lidar_sensor, header.stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f'No transform between {self.frame_map} and '
                        f'{self.frame_lidar_sensor} available with exception: {e}')
            points = np.zeros((0, 3))
            intensities = np.zeros((0,))  # Change here
            trans_map = np.zeros(3)
            rot_map = tf_transformations.quaternion_from_euler(0, 0, 0)  # Identity rotation

        # Apply the transformation to the point cloud data
        rotation_matrix = tf_transformations.quaternion_matrix(rot_map)[:3, :3]
        rotated_points = np.dot(points - trans_map, rotation_matrix.T)

        pointcloud_data = np.zeros(rotated_points.shape[0], dtype=pointcloud_fields)
        pointcloud_data['x'] = rotated_points[:, 0]
        pointcloud_data['y'] = rotated_points[:, 1]
        pointcloud_data['z'] = rotated_points[:, 2]
        pointcloud_data['intensity'] = intensities

        msg = ros_numpy.msgify(PointCloud2, pointcloud_data)
        msg.header = header
        return msg


class VehiclePublisher(BNGPublisher):

    def __init__(self, vehicle,
                 node_name,
                 visualize=True):
        self._vehicle = vehicle
        self.node_name = node_name
        self._sensor_publishers = list()
        
        self._broadcaster_pose = tf2_ros.TransformBroadcaster()
        self.tf_msg = tf2_ros.TransformStamped()
        self.frame_map = 'map'
        self.tf_msg.header.frame_id = self.frame_map
        self.tf_msg.child_frame_id = self._vehicle.vid
        self.alignment_quat = np.array([0, 1, 0, 0])  # sets the forward direction as -y
        # # self.alignment_quat = np.array([0, 0, 0, 1])  # sets the forward direction as -y
        # self.alignment_quat = np.array([1, 0, 0, 0])
        # self.alignment_quat = np.array([0, -1, 0, 0])  # sets the forward direction as -y
        self.current_time = rospy.get_rostime()

        # self.node_name = node_name
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

        self.visualizer = None
        if visualize:
            topic_id = [node_name, vehicle.vid, 'marker']
            topic_id = '/'.join(topic_id)
            self.visualizer = rospy.Publisher(topic_id,
                                              Marker,
                                              queue_size=1)

    def broadcast_vehicle_pose(self, data):
        self.tf_msg.header.stamp = self.current_time
        self.tf_msg.transform.translation.x = data['pos'][0]
        self.tf_msg.transform.translation.y = data['pos'][1]
        self.tf_msg.transform.translation.z = data['pos'][2]
        quat_orientation = np.array([data['rotation'][0],
                                     data['rotation'][1],
                                     data['rotation'][2],
                                     data['rotation'][3]])

        quat_orientation = quaternion_multiply(self.alignment_quat, quat_orientation)
        quat_orientation /= np.linalg.norm(quat_orientation)

        self.tf_msg.transform.rotation.x = quat_orientation[0]  # data['rotation'][0]
        self.tf_msg.transform.rotation.y = quat_orientation[1]  # data['rotation'][1]
        self.tf_msg.transform.rotation.z = quat_orientation[2]  # data['rotation'][2]
        self.tf_msg.transform.rotation.w = quat_orientation[3]  # data['rotation'][3]
        
        # self.tf_msg.transform.rotation.x = data['rotation'][0]
        # self.tf_msg.transform.rotation.y = data['rotation'][1]
        # self.tf_msg.transform.rotation.z = data['rotation'][2]
        # self.tf_msg.transform.rotation.w = data['rotation'][3]
        self._broadcaster_pose.sendTransform(self.tf_msg)


    def state_to_marker(self, data, marker_ns):
        mark = Marker()
        mark.header.frame_id = self.frame_map
        mark.header.stamp = self.current_time
        mark.type = Marker.SPHERE
        mark.ns = marker_ns
        mark.action = Marker.ADD
        mark.id = 0
        mark.lifetime = rospy.Duration()

        mark.pose.position.x = data['pos'][0]
        # mark.pose.position.x = data['pos'][0]+1.9
        mark.pose.position.y = data['pos'][1]
        mark.pose.position.z = data['pos'][2]

        # Apply a 20-degree rotation to the left
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
        self.current_time = current_time
        self._vehicle.poll_sensors()
        self.broadcast_vehicle_pose(self._vehicle.sensors['state'].data)
        for pub in self._sensor_publishers:  # this got us 1fps more
            threading.Thread(target=pub.publish, args=(current_time,), daemon=True).start()
        if self.visualizer is not None:
            mark = self.state_to_marker(self._vehicle.sensors['state'].data, self._vehicle.vid)
            self.visualizer.publish(mark)
            # print ("vehicle marker : ", mark)



# Ori
class NetworkPublisher(BNGPublisher):

    def __init__(self, game_client, node_name):
        self.frame_map = 'map'
        self._game_client = game_client
        self._road_network = None
        self._node_name = node_name
        topic_id = '/'.join([node_name, 'road_network'])
        self._pub = rospy.Publisher(topic_id, MarkerArray, queue_size=1)
        self.current_time = rospy.get_rostime()

    def set_up_road_network_viz(self):
        roads = self._game_client.get_roads()
        
        # Check if roads is a dictionary and not empty
        if isinstance(roads, dict) and roads:
            network_def = dict()
            for r_id, r_inf in roads.items():
                if r_inf['drivability'] != '-1':
                    network_def[int(r_id)] = self._game_client.get_road_edges(r_id)

            self._road_network = MarkerArray()
            for r_id, road in network_def.items():
                # rospy.logdebug(f'++++++++++\nroad: {road}')
                mark = Marker()
                mark.header = std_msgs.msg.Header()
                mark.header.frame_id = self.frame_map
                mark.header.stamp = self.current_time
                mark.type = Marker.LINE_STRIP
                ns = self._node_name
                mark.ns = ns
                mark.action = Marker.ADD
                mark.id = r_id
                mark.lifetime = rospy.Duration(0)  # leave them up forever

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
            rospy.logdebug(f'the road network contains {marker_num} markers')
        else:
            rospy.logwarn("No road information available.")



    def publish(self, current_time):
        self.current_time = current_time
        if self._road_network is None:
            self.set_up_road_network_viz()
        self._pub.publish(self._road_network.markers)


class NetworkPublisherM(BNGPublisher):

    def __init__(self, game_client, node_name):
        self.frame_map = 'map'
        self._game_client = game_client
        self._road_network = None
        self._node_name = node_name
        topic_id = '/'.join([node_name, 'middle_road_network'])
        self._pub = rospy.Publisher(topic_id, MarkerArray, queue_size=1)
        self.current_time = rospy.get_rostime()

    def set_up_road_network_viz(self):
        roads = self._game_client.get_roads()
        network_def = dict()
        for r_id, r_inf in roads.items():
            if r_inf['drivability'] != '-1':
                network_def[int(r_id)] = self._game_client.get_road_edges(r_id)

        self._road_network = MarkerArray()
        for r_id, road in network_def.items():
            # rospy.logdebug(f'++++++++++\nroad: {road}')
            mark = Marker()
            mark.header = std_msgs.msg.Header()
            mark.header.frame_id = self.frame_map
            mark.header.stamp = self.current_time
            mark.type = Marker.LINE_STRIP
            ns = self._node_name
            mark.ns = ns
            mark.action = Marker.ADD
            mark.id = r_id
            mark.lifetime = rospy.Duration(0)  # leave them up forever

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
        marker_num = len(self._road_network.markers)
        rospy.logdebug(f'the road network contains {marker_num} markers')

    def publish(self, current_time):
        self.current_time = current_time
        if self._road_network is None:
            self.set_up_road_network_viz()
        self._pub.publish(self._road_network.markers)

class NetworkPublisherR(BNGPublisher):

    def __init__(self, game_client, node_name):
        self.frame_map = 'map'
        self._game_client = game_client
        self._road_network = None
        self._node_name = node_name
        topic_id = '/'.join([node_name, 'right_road_network'])
        self._pub = rospy.Publisher(topic_id, MarkerArray, queue_size=1)
        self.current_time = rospy.get_rostime()

    def set_up_road_network_viz(self):
        roads = self._game_client.get_roads()
        network_def = dict()
        for r_id, r_inf in roads.items():
            if r_inf['drivability'] != '-1':
                network_def[int(r_id)] = self._game_client.get_road_edges(r_id)

        self._road_network = MarkerArray()
        for r_id, road in network_def.items():
            # rospy.logdebug(f'++++++++++\nroad: {road}')
            mark = Marker()
            mark.header = std_msgs.msg.Header()
            mark.header.frame_id = self.frame_map
            mark.header.stamp = self.current_time
            mark.type = Marker.LINE_STRIP
            ns = self._node_name
            mark.ns = ns
            mark.action = Marker.ADD
            mark.id = r_id
            mark.lifetime = rospy.Duration(0)  # leave them up forever

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
        marker_num = len(self._road_network.markers)
        rospy.logdebug(f'the road network contains {marker_num} markers')

    def publish(self, current_time):
        self.current_time = current_time
        if self._road_network is None:
            self.set_up_road_network_viz()
        self._pub.publish(self._road_network.markers)

class NetworkPublisherL(BNGPublisher):

    def __init__(self, game_client, node_name):
        self.frame_map = 'map'
        self._game_client = game_client
        self._road_network = None
        self._node_name = node_name
        topic_id = '/'.join([node_name, 'left_road_network'])
        self._pub = rospy.Publisher(topic_id, MarkerArray, queue_size=1)
        self.current_time = rospy.get_rostime()

    def set_up_road_network_viz(self):
        roads = self._game_client.get_roads()
        network_def = dict()
        for r_id, r_inf in roads.items():
            if r_inf['drivability'] != '-1':
                network_def[int(r_id)] = self._game_client.get_road_edges(r_id)

        self._road_network = MarkerArray()
        for r_id, road in network_def.items():
            # rospy.logdebug(f'++++++++++\nroad: {road}')
            mark = Marker()
            mark.header = std_msgs.msg.Header()
            mark.header.frame_id = self.frame_map
            mark.header.stamp = self.current_time
            mark.type = Marker.LINE_STRIP
            ns = self._node_name
            mark.ns = ns
            mark.action = Marker.ADD
            mark.id = r_id
            mark.lifetime = rospy.Duration(0)  # leave them up forever

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
        marker_num = len(self._road_network.markers)
        rospy.logdebug(f'the road network contains {marker_num} markers')

    def publish(self, current_time):
        self.current_time = current_time
        if self._road_network is None:
            self.set_up_road_network_viz()
        self._pub.publish(self._road_network.markers)


#Deprecated 
# class IMUPublisher(SensorDataPublisher):

#     def __init__(self, sensor, topic_id):
#         super().__init__(sensor,
#                          topic_id,
#                          sensor_msgs.msg.Imu)

#     def _make_msg(self):
#         data = self._sensor.data
#         msg = sensor_msgs.msg.Imu()
#         msg.orientation = geom_msgs.Quaternion(0, 0, 0, 0)
#         msg.orientation_covariance = [-1, ] * 9
#         msg.angular_velocity = geom_msgs.Vector3(*[data[x] for x in ['aX', 'aY', 'aZ']])
#         msg.angular_velocity_covariance = [-1, ] * 9
#         msg.linear_acceleration = geom_msgs.Vector3(*[data[x] for x in ['gX', 'gY', 'gZ']])
#         msg.linear_acceleration_covariance = [-1, ] * 9
#         return msg


