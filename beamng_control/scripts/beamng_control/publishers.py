from abc import ABC, abstractmethod
import rospy
import numpy as np

np.float = np.float64  # temp fix for following import
import ros_numpy
from sensor_msgs.point_cloud2 import PointCloud2
import tf
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from cv_bridge import CvBridge

import threading

import sensor_msgs
import geometry_msgs.msg as geom_msgs
import std_msgs.msg

import beamngpy.sensors as bng_sensors
# from beamngpy.noise import RandomImageNoise, RandomLIDARNoise

from visualization_msgs.msg import Marker, MarkerArray

import beamng_msgs.msg as bng_msgs
from beamngpy.sensors import Camera


def get_sensor_publisher(sensor):
    sensor_mapping = {
        bng_sensors.State: StatePublisher,
        bng_sensors.Timer: TimerPublisher,
        bng_sensors.Damage: DamagePublisher,
        bng_sensors.GForces: GForcePublisher,
        bng_sensors.IMU: IMUPublisher,
        bng_sensors.Ultrasonic: UltrasonicPublisher,
        bng_sensors.Electrics: ElectricsPublisher,
        bng_sensors.Camera: CameraPublisher,
        bng_sensors.Lidar: LidarPublisher,
        # RandomImageNoise: CameraPublisher,
        # s RandomLIDARNoise: LidarPublisher
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


class IMUPublisher(SensorDataPublisher):

    def __init__(self, sensor, topic_id):
        super().__init__(sensor,
                         topic_id,
                         sensor_msgs.msg.Imu)

    def _make_msg(self):
        data = self._sensor.data
        msg = sensor_msgs.msg.Imu()
        msg.orientation = geom_msgs.Quaternion(0, 0, 0, 0)
        msg.orientation_covariance = [-1, ] * 9
        msg.angular_velocity = geom_msgs.Vector3(*[data[x] for x in ['aX', 'aY', 'aZ']])
        msg.angular_velocity_covariance = [-1, ] * 9
        msg.linear_acceleration = geom_msgs.Vector3(*[data[x] for x in ['gX', 'gY', 'gZ']])
        msg.linear_acceleration_covariance = [-1, ] * 9
        return msg


class UltrasonicPublisher(SensorDataPublisher):

    def __init__(self, sensor, topic_id):
        super().__init__(sensor,
                         topic_id,
                         bng_msgs.USSensor)

    def _make_msg(self):
        data = self._sensor.data
        msg = bng_msgs.USSensor()
        msg.distance = data['distance']
        return msg


class ElectricsPublisher(SensorDataPublisher):

    def __init__(self, sensor, topic_id):
        super().__init__(sensor,
                         topic_id,
                         bng_msgs.ElectricsSensor)

    def _make_msg(self):
        data = self._sensor.data
        msg = bng_msgs.ElectricsSensor()
        msg.engine_running = data['engineRunning']
        msg.virtual_airspeed = data['virtualAirspeed']
        msg.lights = data['lights']
        msg.turnsignal = data['turnsignal']
        msg.altitude = data['altitude']
        msg.brake = data['brake']
        msg.throttle = data['throttle']
        msg.running = data['running']
        msg.rpm_spin = data['rpmspin']
        msg.is_TC_brake_active = data['isTCBrakeActive']
        for k, v in data['wheelThermals'].items():
            info = bng_msgs.WheelThermalsInfo()
            info.id = k
            info.brake_surface_temperature = v['brakeSurfaceTemperature']
            info.brake_thermalefficiency = v['brakeThermalEfficiency']
            info.brake_core_temperature = v['brakeCoreTemperature']
            msg.wheel_thermals.append(info)
        msg.parkingbrake_input = data['parkingbrake_input']
        msg.parking = data['parking']
        msg.clutch_input = data['clutch_input']
        msg.hazard = data['hazard']
        msg.oil = data['oil']
        msg.low_high_beam = data['lowhighbeam']
        msg.low_beam = data['lowbeam']
        msg.high_beam = data['highbeam']
        msg.lightbar = data['lightbar']
        msg.wheel_speed = data['wheelspeed']
        msg.tcs = data['tcs']
        msg.airflow_speed = data['airflowspeed']
        msg.ignition = data['ignition']
        msg.dse_color = data['dseColor']
        msg.airspeed = data['airspeed']
        msg.rpm = data['rpm']
        msg.reverse = data['reverse']
        msg.clutch = data['clutch']
        msg.abs = data['abs']
        msg.steering = data['steering']
        msg.horn = data['horn']
        msg.driveshaft = data['driveshaft']
        msg.steering_input = data['steering_input']
        msg.fuel = data['fuel']
        msg.has_ABS = data['hasABS']
        msg.parkingbrake = data['parkingbrake']
        msg.lowpressure = data['lowpressure']
        msg.throttle_input = data['throttle_input']
        msg.brake_input = data['brake_input']
        msg.is_YC_brake_active = data['isYCBrakeActive']
        msg.lowfuel = data['lowfuel']
        msg.gear = data['gear']
        msg.esc = data['esc']
        msg.abs_active = data['abs_active']
        msg.avg_wheel_av = data['avg_wheel_av']
        msg.check_engine = data['check_engine']
        msg.engine_load = data['engine_load']
        msg.engine_throttle = data['engine_throttle']
        msg.esc_active = data['esc_active']
        msg.exhaust_flow = data['exhaust_flow']
        msg.fog_lights = data['fog_lights']
        msg.fuel_volume = data['fuel_volume']
        msg.fuel_capacity = data['fuel_capacity']
        msg.gear_position = data['gear_a']
        msg.gear_index = data['gear_index']
        msg.gear_m = data['gear_m']
        msg.hazard_signal = data['hazard_signal']
        msg.is_shifting = data['is_shifting']
        msg.headlights = data['headlights']
        msg.oil_temperature = data['oil_temperature']
        msg.radiator_fan_spin = data['radiator_fan_spin']
        msg.rpm_tacho = data['rpm_tacho']
        msg.signal_l = data['signal_l']
        msg.left_signal = data['left_signal']
        msg.signal_r = data['signal_r']
        msg.right_signal = data['right_signal']
        msg.tcs_active = data['tcs_active']
        msg.water_temperature = data['water_temperature']
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
                         sensor_msgs.msg.Image)
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
                         sensor_msgs.msg.Image)
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
                         sensor_msgs.msg.Image)
        self._cv_helper = cv_helper
        self._vehicle = vehicle
        self._classes = None

    def _update_data_with_bbox(self, data):
        if self._classes is None:
            annotations = self._vehicle.bng.get_annotations()
            self._classes = self._vehicle.bng.get_annotation_classes(annotations)
        bboxes = Camera.extract_bounding_boxes(data['annotation'],
                                               data['instance'],
                                               self._classes)
        bboxes = [b for b in bboxes if b['class'] == 'CAR']
        rospy.logdebug(f'bboxes: {bboxes}')
        bbox_img = Camera.draw_bounding_boxes(bboxes,
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
        if self._sensor.bbox:
            bbox_topic = '/'.join([topic_id, 'bounding_box'])
            pub = BBoxImgPublisher(sensor,
                                   bbox_topic,
                                   self._cv_helper,
                                   vehicle)
            self._publishers.append(pub)

    def publish(self, current_time):
        if self._sensor.is_render_instance:
            data = self._sensor.get_full_poll_request()
        else:
            data = self._sensor.poll()
        for pub in self._publishers:
            pub.current_time = current_time
            pub.publish(current_time, data)


class LidarPublisher(SensorDataPublisher):

    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, sensor_msgs.msg.PointCloud2)
        self.listener = tf.TransformListener()
        self.frame_lidar_sensor = 'lidar_link'

    def _make_msg(self):
        header = std_msgs.msg.Header()
        header.frame_id = self.frame_lidar_sensor
        header.stamp = self.current_time

        readings_data = self._sensor.poll()
        points = np.array(readings_data['pointCloud'])
        colours = readings_data['colours']

        pointcloud_fields = [('x', np.float32),
                             ('y', np.float32),
                             ('z', np.float32),
                             ('intensity', np.float32)]

        try:
            (trans_map, _) = self.listener.lookupTransform(self.frame_map, self.frame_lidar_sensor, header.stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f'No transform between {self.frame_map} and '
                          f'{self.frame_lidar_sensor} available with exception: {e}')
            points = np.zeros((0, 3))
            colours = np.zeros((0,))
            trans_map = np.zeros(3)
        pointcloud_data = np.zeros(points.shape[0], dtype=pointcloud_fields)
        pointcloud_data['x'] = points[:, 0] - trans_map[0]
        pointcloud_data['y'] = points[:, 1] - trans_map[1]
        pointcloud_data['z'] = points[:, 2] - trans_map[2]
        pointcloud_data['intensity'] = np.array(colours)
        msg = ros_numpy.msgify(PointCloud2, pointcloud_data)
        msg.header = header
        return msg


class VehiclePublisher(BNGPublisher):

    def __init__(self, vehicle,
                 node_name,
                 visualize=True):
        self._vehicle = vehicle
        self.node_name = node_name
        self._broadcaster_pose = tf2_ros.TransformBroadcaster()
        self.tf_msg = tf2_ros.TransformStamped()
        self.frame_map = 'map'
        self.tf_msg.header.frame_id = self.frame_map
        self.tf_msg.child_frame_id = self._vehicle.vid
        self.current_time = rospy.get_rostime()

        self.node_name = node_name
        self._sensor_publishers = list()
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

        self.tf_msg.transform.rotation.x = data['rotation'][0]
        self.tf_msg.transform.rotation.y = data['rotation'][1]
        self.tf_msg.transform.rotation.z = data['rotation'][2]
        self.tf_msg.transform.rotation.w = data['rotation'][3]
        self._broadcaster_pose.sendTransform(self.tf_msg)

    def state_to_marker(self, data, marker_ns):
        mark = Marker()
        mark.header.frame_id = self.frame_map
        mark.header.stamp = self.current_time
        mark.type = Marker.CUBE
        mark.ns = marker_ns
        mark.action = Marker.ADD
        mark.id = 0
        mark.lifetime = rospy.Duration()

        mark.pose.position.x = data['pos'][0]
        mark.pose.position.y = data['pos'][1]
        mark.pose.position.z = data['pos'][2]
        mark.pose.orientation.x = data['rotation'][0]
        mark.pose.orientation.y = data['rotation'][1]
        mark.pose.orientation.z = data['rotation'][2]
        mark.pose.orientation.w = data['rotation'][3]

        mark.scale.x = 5
        mark.scale.y = 1.9
        mark.scale.z = 1.5

        mark.color.r = 0.0
        mark.color.g = 1.0
        mark.color.b = 0.0
        mark.color.a = 1.0

        return mark

    def publish(self, current_time):
        self.current_time = current_time
        self._vehicle.poll_sensors()
        self.broadcast_vehicle_pose(self._vehicle.sensors['state'].data)
        for pub in self._sensor_publishers:  # this got us 1fps more
            threading.Thread(target=pub.publish, args=(current_time,), daemon=True).start()
        if self.visualizer is not None:
            mark = self.state_to_marker(self._vehicle.sensors['state'].data,
                                        self._vehicle.vid)
            self.visualizer.publish(mark)


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
        network_def = dict()
        for r_id, r_inf in roads.items():
            if r_inf['drivability'] != '-1':
                network_def[int(r_id)] = self._game_client.get_road_edges(r_id)

        self._road_network = MarkerArray()
        for r_id, road in network_def.items():
            rospy.logdebug(f'++++++++++\nroad: {road}')
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

            mark.scale.x = 2
            mark.scale.y = 1

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

    def publish(self, current_time):
        self.current_time = current_time
        if self._road_network is None:
            self.set_up_road_network_viz()
        self._pub.publish(self._road_network.markers)
