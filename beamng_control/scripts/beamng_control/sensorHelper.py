import rospy
import copy
import numpy as np

import beamngpy.sensors as bng_sensors
from beamng_control.publishers import LidarPublisher, CameraPublisher


# from beamngpy.noise import RandomImageNoise, RandomLIDARNoise


class SensorSpecificationError(TypeError):
    """
    Raised if any non-optional argument is not given for sensor definition.
    """
    pass


def get_lidar(bng,
              vehicle,
              position,
              rotation,
              vertical_resolution,
              vertical_angle,
              max_distance,
              **spec):
    if 'shmem' in spec and spec['shmem']:
        spec.pop('shmem')
        rospy.logwarn('The Lidar sensor provides no shared memory support, '
                      'sensor data is always shared through sockets.')
    try:
        lidar = bng_sensors.Lidar(bng=bng,
                                  vehicle=vehicle,
                                  pos=position,
                                  dir=rotation,
                                  vertical_resolution=vertical_resolution,
                                  vertical_angle=vertical_angle,
                                  max_distance=max_distance,
                                  is_using_shared_memory=False,
                                  **spec)
    except TypeError as e:
        raise SensorSpecificationError('Could not get Lidar instance, the '
                                       'json specification provided an'
                                       'unexpected input. List of possible'
                                       f'unexpected inputs:\n{spec}\n'
                                       'Original error '
                                       f'message:\n{e}')
    return lidar


def get_imu(position=None, node=None, **spec):
    if not (bool(position is None) ^ bool(node is None)):
        rospy.logerr('The IMU sensor definition needs to specify either'
                     ' a node or a position for the sensor placement.')
    try:
        imu = bng_sensors.IMU(node=node, pos=position)
    except TypeError as e:
        raise SensorSpecificationError('Could not get IMU instance, the '
                                       'json specification provided an'
                                       'unexpected input. List of possible'
                                       f'unexpected inputs:\n{spec}\n'
                                       '\nOriginal error '
                                       f'message:\n{e}')
    return imu


def get_ultrasonic(position, rotation, **spec):
    spec['near_far'] = (spec.pop('min_distance'),
                        spec.pop('max_distance'))
    try:
        us = bng_sensors.Ultrasonic(position,
                                    rotation,
                                    **spec)
    except TypeError as e:
        raise SensorSpecificationError('Could not get ultrasonic sensor '
                                       'instance, the json specification '
                                       'provided an unexpected input. List '
                                       'of possible unexpected inputs:'
                                       f'\n{spec}\nOriginal error '
                                       f'message:\n{e}')
    return us


def rotate_camera(vec, pitch, yaw):
    R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0]
                        [-np.sin(pitch), 0, np.cos(pitch)]])
    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0],
                      [0, 0, 1]])
    return R_yaw @ R_pitch @ vec


def get_bounding_box_center(bbox):
    rbl = np.array(bbox['rear_bottom_left'])
    fbr = np.array(bbox['front_bottom_right'])
    rtl = np.array([bbox['rear_top_left']])
    center = rbl + ((fbr - rbl) / 2) + ((rtl - rbl) / 2)
    return center


def get_camera(name, bng, vehicle, position, rotation, field_of_view_y, resolution, **spec):
    bbox = spec.pop('bounding_box')
    if 'is_using_shared_memory' in spec:
        rospy.logerr('Shared memory is automatically disabled '
                     'for the camera sensor.')
        spec.pop('is_using_shared_memory')

    try:
        cam = bng_sensors.Camera(name=name,
                                 bng=bng,
                                 vehicle=vehicle,
                                 pos=position,
                                 dir=(0, -1, 0),
                                 up = (0, 0, 1),
                                 field_of_view_y=field_of_view_y,
                                 resolution=resolution,
                                 is_using_shared_memory=False,
                                 **spec)
        center = vehicle.get_bounding_box_center(vehicle.get_bbox())
        camera_direction = center - rotate_camera(position, rotation[0], rotation[1])
        cam.set_direction((camera_direction[0], camera_direction[1], camera_direction[2]))


    except TypeError as e:
        raise SensorSpecificationError('Could not get Camera instance, the '
                                       'json specification provided an'
                                       'unexpected input. List of possible'
                                       f'unexpected inputs:\n{spec}\n'
                                       '\nOriginal error '
                                       f'message:\n{e}')
    if bbox and not (cam.is_render_instance and cam.is_render_annotations):
        rospy.logerr('Enabled annotations and instance annotations'
                     'are required to generate images with bounding box.')
    else:
        cam.bbox = bbox
    return cam


def get_camera_noise_sensor(sensor, **spec):
    try:
        noise = RandomImageNoise(sensor, **spec)
    except TypeError as e:
        raise SensorSpecificationError('Could not get RandomImageNoise'
                                       'instance, the json '
                                       'specification provided an'
                                       'unexpected input. List of possible'
                                       f'unexpected inputs:\n{spec}\n'
                                       '\nOriginal error '
                                       f'message:\n{e}')
    return noise


def get_lidar_noise_sensor(sensor, **spec):
    try:
        noise = RandomLIDARNoise(sensor, **spec)
    except TypeError as e:
        raise SensorSpecificationError('Could not get RandomImageNoise'
                                       'instance, the json '
                                       'specification provided an'
                                       'unexpected input. List of possible'
                                       f'unexpected inputs:\n{spec}\n'
                                       '\nOriginal error '
                                       f'message:\n{e}')
    return noise


def select_sensor_definition(sensor_type_name, sensor_defs):
    """
    Returns type of sensor (f.ex. 'imu')
    and a non vehicle specific sensor definition (f.ex. camera resolution etc.)
    """
    sensor_type = sensor_type_name.split('.')
    rospy.logdebug(f'sensor_type:{sensor_type}')
    sensor_spec = copy.deepcopy(sensor_defs)
    for t in sensor_type:
        rospy.logdebug(f'sensor type key: {t}')
        if t not in sensor_spec:
            return sensor_type_name, None
        sensor_spec = sensor_spec[t]
    return sensor_type[0], sensor_spec


def get_sensor(sensor_type, all_sensor_defs, bng=None, vehicle=None, name=None, dyn_sensor_properties=None):
    """
    Args:
    sensor_type(string): used to look up static part of sensor definition
    all_sensor_defs(dict): containing definitions for all sensors for
    attributes that multiple sensors may share across different vehicles
    dyn_sensor_properties(dict): attributes that vary per sensor model,
    f.ex. position, rotation, etc.
    """
    global _sensor_getters
    sensor_class_name, static_sensor_def = select_sensor_definition(sensor_type, all_sensor_defs)
    if sensor_class_name not in _sensor_getters:
        rospy.logerr(f'Sensor of type {sensor_class_name} not available.')
    sensor_def = dict()
    if static_sensor_def:
        sensor_def.update(static_sensor_def)
    if dyn_sensor_properties:
        sensor_def.update(dyn_sensor_properties)
    rospy.logdebug(f'sensor_def: {sensor_def}')
    try:
        if sensor_class_name in _automation_sensors:
            sensor = _sensor_getters[sensor_class_name](bng=bng,
                                                        name=name,
                                                        vehicle=vehicle,
                                                        **sensor_def)
            sensor_publisher = _sensor_automation_type_publisher_getter[sensor_class_name]
        else:
            sensor = _sensor_getters[sensor_class_name](**sensor_def)
            sensor_publisher = None
    except TypeError as err:
        raise SensorSpecificationError(f'The {sensor_class_name} sensor '
                                       'definition is missing one or more '
                                       'fields. These fields '
                                       f'where defined:\n{sensor_def}\n'
                                       f'Original error message:\n{err}')
    return sensor, sensor_publisher


_automation_sensors = ['Camera',
                       'Lidar',
                       'CameraNoise',
                       'LidarNoise',
                       'Ultrasonic',
                       'AdvancedIMU',  # unsure about this one
                       'PowerTrain',  # unsure about this one
                       'Radar',  # unsure about this one
                       'meshsensor',  # unsure about this one
                       ]
_sensor_automation_type_publisher_getter = {
    'Lidar': LidarPublisher,
    'Camera': CameraPublisher,
}

_sensor_getters = {
    'IMU': get_imu,
    'Damage': bng_sensors.Damage,
    'Timer': bng_sensors.Timer,
    'GForces': bng_sensors.GForces,
    'Ultrasonic': get_ultrasonic,
    'Electrics': bng_sensors.Electrics,
    'Camera': get_camera,
    'Lidar': get_lidar,
    'CameraNoise': get_camera_noise_sensor,
    'LidarNoise': get_lidar_noise_sensor
}
