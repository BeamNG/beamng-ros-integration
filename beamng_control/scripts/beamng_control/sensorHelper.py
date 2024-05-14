import rospy
import copy

import beamngpy.sensors as bng_sensors
from beamng_control.publishers import LidarPublisher, CameraPublisher, UltrasonicPublisher, AdvancedIMUPublisher


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
    if 'is_using_shared_memory' in spec:
        spec['is_using_shared_memory'] = False
        rospy.loginfo('Shared memory is automatically disabled '
                     'for the Lidar sensor.')

    try:
        lidar = bng_sensors.Lidar(bng=bng,
                                  vehicle=vehicle,
                                  pos=position,
                                  dir=rotation,
                                  vertical_resolution=vertical_resolution,
                                  vertical_angle=vertical_angle,
                                  max_distance=max_distance,
                                  **spec)
    except TypeError as e:
        raise SensorSpecificationError('Could not get Lidar instance, the '
                                       'json specification provided an'
                                       'unexpected input. List of possible'
                                       f'unexpected inputs:\n{spec}\n'
                                       'Original error '
                                       f'message:\n{e}')
    return lidar




def get_ultrasonic(bng,
                   name,
                   vehicle,
                   position,
                   rotation,
                   range_min_cutoff, 
                   range_direct_max_cutoff,
                   **spec):
    if 'is_streaming' in spec:
        spec['is_streaming'] = False
        rospy.loginfo('Shared memory is automatically disabled '
                     'for the Ultrasonic sensor.')


    try:
        us = bng_sensors.Ultrasonic(bng=bng,
                                    name=name,
                                    vehicle=vehicle,
                                    pos=position,
                                    dir=rotation,
                                    range_min_cutoff=range_min_cutoff, 
                                    range_direct_max_cutoff=range_direct_max_cutoff,
                                    **spec)
    except TypeError as e:
        raise SensorSpecificationError('Could not get ultrasonic sensor '
                                       'instance, the json specification '
                                       'provided an unexpected input. List '
                                       'of possible unexpected inputs:'
                                       f'\n{spec}\nOriginal error '
                                       f'message:\n{e}')
    return us



def get_advanced_imu(bng,
                    name,
                    vehicle,
                    position,
                    rotation,
                    **spec):
    try:
        AdImu = bng_sensors.AdvancedIMU(bng=bng,
                                    name=name,
                                    vehicle=vehicle,
                                    pos=position,
                                    dir=rotation,
                                    **spec)
    except TypeError as e:
        raise SensorSpecificationError(f'Could not get AdvancedIMU instance. '
                                       f'Original error message:\n{e}')
    return AdImu



def get_camera(name, bng, vehicle, position, rotation, field_of_view_y, resolution, **spec):
    # if 'bounding_box' in spec:
    #     if spec['bounding_box']:
    #         spec['bounding_box'] = False
    #         rospy.logerr('Bounding boxes are not supported '
    #                      'for the camera sensor.')

    if 'is_using_shared_memory' in spec:
        spec['is_using_shared_memory'] = False
        rospy.loginfo('Shared memory is automatically disabled '
                     'for the camera sensor.')
    try:
        cam = bng_sensors.Camera(name=name,
                                 bng=bng,
                                 vehicle=vehicle,
                                 pos=position,
                                 dir=rotation,
                                 field_of_view_y=field_of_view_y,
                                 resolution=resolution,
                                 **spec)
    except TypeError as e:
        raise SensorSpecificationError('Could not get Camera instance, the '
                                       'json specification provided an'
                                       'unexpected input. List of possible'
                                       f'unexpected inputs:\n{spec}\n'
                                       '\nOriginal error '
                                       f'message:\n{e}')
    # if bbox and not (cam.is_render_instance and cam.is_render_annotations):
    #     rospy.logerr('Enabled annotations and instance annotations'
    #                  'are required to generate images with bounding box.')
    # else:
    #     cam.bbox = bbox
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

def get_sensors_classical(sensor_type, all_sensor_defs, dyn_sensor_properties=None):
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
    try:
        sensor = _sensor_getters[sensor_class_name](**sensor_def)
    except TypeError as err:
        raise SensorSpecificationError(f'The {sensor_class_name} sensor '
                                       'definition is missing one or more '
                                       'fields. These fields '
                                       f'where defined:\n{sensor_def}\n'
                                       f'Original error message:\n{err}')
    return sensor



def get_sensors_automation(sensor_type, all_sensor_defs, bng=None, vehicle=None, name=None, dyn_sensor_properties=None):
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
                   'AdvancedIMU',  
                   'Ultrasonic',
                   'PowerTrain',  # unsure about this one
                   'Radar',  # unsure about this one
                   'meshsensor',  # unsure about this one
                  ]
_sensor_automation_type_publisher_getter = {
    'Lidar': LidarPublisher,
    'Camera': CameraPublisher,
    'Ultrasonic': UltrasonicPublisher,
    'AdvancedIMU': AdvancedIMUPublisher
}

_sensor_getters = {
    'Damage': bng_sensors.Damage,
    'Timer': bng_sensors.Timer,
    'GForces': bng_sensors.GForces,
    'Electrics': bng_sensors.Electrics,
    'IMU': get_imu,
    
    'Camera': get_camera,
    'Ultrasonic': get_ultrasonic,
    'Lidar': get_lidar,
    'AdvancedIMU': get_advanced_imu
    
    # 'Ultrasonic': bng_sensors.Ultrasonic,
    # 'CameraNoise': get_camera_noise_sensor,
    # 'LidarNoise': get_lidar_noise_sensor
}