import rospy
import copy

import beamngpy.sensors as bng_sensors
from beamng_control.publishers import LidarPublisher, CameraPublisher, UltrasonicPublisher, AdvancedIMUPublisher, GPSPublisher, MeshPublisher, IdealRadarPublisher, PowertrainSensorPublisher, RadarPublisher, RoadsSensorPublisher


class SensorSpecificationError(TypeError):
    """
    Raised if any non-optional argument is not given for sensor definition.
    """
    pass


def get_lidar(bng, vehicle, position, rotation, vertical_resolution, vertical_angle, max_distance, **spec):
    """
    Initializes a Lidar sensor.

    Args:
        bng (BeamNGpy): BeamNGpy instance used for communication.
        vehicle (Vehicle): The vehicle to which the sensor is attached.
        position (tuple): Position of the sensor.
        rotation (tuple): Rotation of the sensor.
        vertical_resolution (int): Vertical resolution of the Lidar sensor.
        vertical_angle (float): Vertical angle of the Lidar sensor.
        max_distance (float): Maximum distance the Lidar sensor can measure.
        **spec (dict): Additional sensor specifications.
    
    Returns:
        Lidar: An instance of the Lidar sensor.

    Raises:
        SensorSpecificationError: If sensor initialization fails due to incorrect specifications.
    """
    if 'is_using_shared_memory' in spec:
        spec['is_using_shared_memory'] = False
        rospy.loginfo('Shared memory is automatically disabled for the Lidar sensor.')

    try:
        lidar = bng_sensors.Lidar(bng=bng, vehicle=vehicle, pos=position, dir=rotation,
                                  vertical_resolution=vertical_resolution, vertical_angle=vertical_angle,
                                  max_distance=max_distance, **spec)
    except TypeError as e:
        raise SensorSpecificationError(f'Could not get Lidar instance, unexpected input:\n{spec}\nError: {e}')
    return lidar


def get_ultrasonic(bng, name, vehicle, position, rotation, range_min_cutoff, range_direct_max_cutoff, **spec):
    """
    Initializes an Ultrasonic sensor.

    Args:
        bng (BeamNGpy): BeamNGpy instance used for communication.
        name (str): Name of the ultrasonic sensor.
        vehicle (Vehicle): The vehicle to which the sensor is attached.
        position (tuple): Position of the sensor.
        rotation (tuple): Rotation of the sensor.
        range_min_cutoff (float): Minimum range cutoff for the sensor.
        range_direct_max_cutoff (float): Maximum direct range cutoff.
        **spec (dict): Additional sensor specifications.
    
    Returns:
        Ultrasonic: An instance of the Ultrasonic sensor.

    Raises:
        SensorSpecificationError: If sensor initialization fails due to incorrect specifications.
    """
    if 'is_streaming' in spec:
        spec['is_streaming'] = False
        rospy.loginfo('Shared memory is automatically disabled for the Ultrasonic sensor.')

    try:
        us = bng_sensors.Ultrasonic(bng=bng, name=name, vehicle=vehicle, pos=position, dir=rotation,
                                    range_min_cutoff=range_min_cutoff, range_direct_max_cutoff=range_direct_max_cutoff,
                                    **spec)
    except TypeError as e:
        raise SensorSpecificationError(f'Could not get Ultrasonic sensor instance. Error: {e}')
    return us


def get_advanced_imu(bng, name, vehicle, position, rotation, **spec):
    """
    Initializes an Advanced IMU sensor.

    Args:
        bng (BeamNGpy): BeamNGpy instance used for communication.
        name (str): Name of the Advanced IMU sensor.
        vehicle (Vehicle): The vehicle to which the sensor is attached.
        position (tuple): Position of the sensor.
        rotation (tuple): Rotation of the sensor.
        **spec (dict): Additional sensor specifications.
    
    Returns:
        AdvancedIMU: An instance of the Advanced IMU sensor.

    Raises:
        SensorSpecificationError: If sensor initialization fails due to incorrect specifications.
    """
    try:
        AdImu = bng_sensors.AdvancedIMU(bng=bng, name=name, vehicle=vehicle, pos=position, dir=rotation, **spec)
    except TypeError as e:
        raise SensorSpecificationError(f'Could not get AdvancedIMU instance. Error: {e}')
    return AdImu


def get_gps(bng, name, vehicle, position, rotation, **spec):
    """
    Initializes a GPS sensor.

    Args:
        bng (BeamNGpy): BeamNGpy instance used for communication.
        name (str): Name of the GPS sensor.
        vehicle (Vehicle): The vehicle to which the sensor is attached.
        position (tuple): Position of the sensor.
        rotation (tuple): Rotation of the sensor.
        **spec (dict): Additional sensor specifications.
    
    Returns:
        GPS: An instance of the GPS sensor.

    Raises:
        SensorSpecificationError: If sensor initialization fails due to incorrect specifications.
    """
    if 'rotation' in spec:
        spec.pop('rotation')
        rospy.loginfo('GPS sensor has no rotation')
    try:
        Gps = bng_sensors.GPS(bng=bng, name=name, vehicle=vehicle, pos=position)
    except TypeError as e:
        raise SensorSpecificationError(f'Could not get GPS instance. Error: {e}')
    return Gps


def get_ideal_radar(bng, name, vehicle, position, rotation, **spec):
    """
    Initializes an IdealRadar sensor.

    Args:
        bng (BeamNGpy): BeamNGpy instance used for communication.
        name (str): Name of the IdealRadar sensor.
        vehicle (Vehicle): The vehicle to which the sensor is attached.
        position (tuple): Position of the sensor.
        rotation (tuple): Rotation of the sensor.
        **spec (dict): Additional sensor specifications.
    
    Returns:
        IdealRadar: An instance of the IdealRadar sensor.

    Raises:
        SensorSpecificationError: If sensor initialization fails due to incorrect specifications.
    """
    try:
        ideal_radar = bng_sensors.IdealRadar(bng=bng, name=name, vehicle=vehicle)
    except TypeError as e:
        raise SensorSpecificationError(f'Could not get IdealRadar instance. Error: {e}')
    return ideal_radar
def get_radar(bng, name, vehicle, position, rotation, **spec):
    """
    Initializes a Radar sensor.

    Args:
        bng (BeamNGpy): BeamNGpy instance used for communication.
        name (str): Name of the Radar sensor.
        vehicle (Vehicle): The vehicle to which the sensor is attached.
        position (tuple): Position of the sensor.
        rotation (tuple): Rotation of the sensor.
        **spec (dict): Additional sensor specifications.
    
    Returns:
        Radar: An instance of the Radar sensor.

    Raises:
        SensorSpecificationError: If sensor initialization fails due to incorrect specifications.
    """
    try:
        radar = bng_sensors.Radar(bng=bng, name=name, vehicle=vehicle)
    except TypeError as e:
        raise SensorSpecificationError(f'Could not get Radar instance. Error: {e}')
    return radar


def get_roads_sensor(bng, name, vehicle, position, **spec):
    """
    Initializes a RoadsSensor.

    Args:
        bng (BeamNGpy): BeamNGpy instance used for communication.
        name (str): Name of the RoadsSensor.
        vehicle (Vehicle): The vehicle to which the sensor is attached.
        position (tuple): Position of the sensor.
        **spec (dict): Additional sensor specifications.
    
    Returns:
        RoadsSensor: An instance of the RoadsSensor.

    Raises:
        SensorSpecificationError: If sensor initialization fails due to incorrect specifications.
    """
    if 'rotation' in spec:
        spec.pop('rotation')
        rospy.loginfo('RoadsSensor has no rotation.')
    if 'position' in spec:
        spec.pop('position')
        rospy.loginfo('RoadsSensor has no position.')
    try:
        roads_sensor = bng_sensors.RoadsSensor(bng=bng, name=name, vehicle=vehicle)
    except TypeError as e:
        raise SensorSpecificationError(f'Could not get Roads Sensor instance. Error: {e}')
    return roads_sensor


def get_power_train(bng, name, vehicle, position, rotation, **spec):
    """
    Initializes a PowertrainSensor.

    Args:
        bng (BeamNGpy): BeamNGpy instance used for communication.
        name (str): Name of the PowertrainSensor.
        vehicle (Vehicle): The vehicle to which the sensor is attached.
        position (tuple): Position of the sensor.
        rotation (tuple): Rotation of the sensor.
        **spec (dict): Additional sensor specifications.
    
    Returns:
        PowertrainSensor: An instance of the PowertrainSensor.

    Raises:
        SensorSpecificationError: If sensor initialization fails due to incorrect specifications.
    """
    try:
        power_train_sensor = bng_sensors.PowertrainSensor(bng=bng, name=name, vehicle=vehicle)
    except TypeError as e:
        raise SensorSpecificationError(f'Could not get Powertrain Sensor instance. Error: {e}')
    return power_train_sensor


def get_mesh(bng, name, vehicle, position, rotation, **spec):
    """
    Initializes a Mesh sensor.

    Args:
        bng (BeamNGpy): BeamNGpy instance used for communication.
        name (str): Name of the Mesh sensor.
        vehicle (Vehicle): The vehicle to which the sensor is attached.
        position (tuple): Position of the sensor.
        rotation (tuple): Rotation of the sensor.
        **spec (dict): Additional sensor specifications.
    
    Returns:
        Mesh: An instance of the Mesh sensor.

    Raises:
        SensorSpecificationError: If sensor initialization fails due to incorrect specifications.
    """
    try:
        mesh = bng_sensors.Mesh(bng=bng, name=name, vehicle=vehicle, **spec)
    except TypeError as e:
        raise SensorSpecificationError(f'Could not get Mesh instance. Error: {e}')
    return mesh


def get_camera(name, bng, vehicle, position, rotation, field_of_view_y, resolution, **spec):
    """
    Initializes a Camera sensor.

    Args:
        name (str): Name of the Camera sensor.
        bng (BeamNGpy): BeamNGpy instance used for communication.
        vehicle (Vehicle): The vehicle to which the sensor is attached.
        position (tuple): Position of the sensor.
        rotation (tuple): Rotation of the sensor.
        field_of_view_y (float): The vertical field of view for the camera.
        resolution (tuple): Resolution of the camera (width, height).
        **spec (dict): Additional sensor specifications.
    
    Returns:
        Camera: An instance of the Camera sensor.

    Raises:
        SensorSpecificationError: If sensor initialization fails due to incorrect specifications.
    """
    if 'is_using_shared_memory' in spec:
        spec['is_using_shared_memory'] = False
        rospy.loginfo('Shared memory is automatically disabled for the camera sensor.')
    try:
        cam = bng_sensors.Camera(name=name, bng=bng, vehicle=vehicle, pos=position, dir=rotation,
                                 field_of_view_y=field_of_view_y, resolution=resolution, **spec)
    except TypeError as e:
        raise SensorSpecificationError(f'Could not get Camera instance. Error: {e}')
    return cam


def get_camera_noise_sensor(sensor, **spec):
    """
    Adds noise to a Camera sensor.

    Args:
        sensor (Camera): The camera sensor instance.
        **spec (dict): Additional noise specifications.

    Returns:
        RandomImageNoise: An instance of the noise to be applied to the camera.

    Raises:
        SensorSpecificationError: If noise initialization fails due to incorrect specifications.
    """
    try:
        noise = RandomImageNoise(sensor, **spec)
    except TypeError as e:
        raise SensorSpecificationError(f'Could not get RandomImageNoise instance. Error: {e}')
    return noise


def get_lidar_noise_sensor(sensor, **spec):
    """
    Adds noise to a Lidar sensor.

    Args:
        sensor (Lidar): The Lidar sensor instance.
        **spec (dict): Additional noise specifications.

    Returns:
        RandomLIDARNoise: An instance of the noise to be applied to the Lidar.

    Raises:
        SensorSpecificationError: If noise initialization fails due to incorrect specifications.
    """
    try:
        noise = RandomLIDARNoise(sensor, **spec)
    except TypeError as e:
        raise SensorSpecificationError(f'Could not get RandomLIDARNoise instance. Error: {e}')
    return noise


def get_imu(position=None, node=None, **spec):
    """
    Initializes an IMU sensor.

    Args:
        position (tuple, optional): Position of the sensor.
        node (str, optional): Node to which the sensor is attached.
        **spec (dict): Additional sensor specifications.
    
    Returns:
        IMU: An instance of the IMU sensor.

    Raises:
        SensorSpecificationError: If sensor initialization fails due to incorrect specifications.
    """
    if not (bool(position is None) ^ bool(node is None)):
        rospy.logerr('The IMU sensor definition needs to specify either a node or a position for the sensor placement.')
    try:
        imu = bng_sensors.IMU(node=node, pos=position)
    except TypeError as e:
        raise SensorSpecificationError(f'Could not get IMU instance. Error: {e}')
    return imu


def select_sensor_definition(sensor_type_name, sensor_defs):
    """
    Selects a sensor definition based on the sensor type.

    Args:
        sensor_type_name (str): The name of the sensor type.
        sensor_defs (dict): Dictionary containing sensor definitions.

    Returns:
        tuple: Sensor type and its corresponding sensor specification.

    """
    sensor_type = sensor_type_name.split('.')
    rospy.logdebug(f'Sensor type: {sensor_type}')
    sensor_spec = copy.deepcopy(sensor_defs)
    for t in sensor_type:
        rospy.logdebug(f'Sensor type key: {t}')
        if t not in sensor_spec:
            return sensor_type_name, None
        sensor_spec = sensor_spec[t]
    return sensor_type[0], sensor_spec


def get_sensors_classical(sensor_type, all_sensor_defs, dyn_sensor_properties=None):
    """
    Retrieves a sensor with classical definitions.

    Args:
        sensor_type (str): Type of sensor (e.g., 'imu').
        all_sensor_defs (dict): Dictionary of all sensor definitions.
        dyn_sensor_properties (dict, optional): Dynamic sensor properties.

    Returns:
        sensor: The initialized sensor instance.

    Raises:
        SensorSpecificationError: If the sensor definition is incomplete or missing fields.
    """
    global _sensor_getters
    sensor_class_name, static_sensor_def = select_sensor_definition(sensor_type, all_sensor_defs)
    if sensor_class_name not in _sensor_getters:
        rospy
        rospy.logerr(f'Sensor of type {sensor_class_name} not available.')

    sensor_def = dict()
    if static_sensor_def:
        sensor_def.update(static_sensor_def)
    if dyn_sensor_properties:
        sensor_def.update(dyn_sensor_properties)

    try:
        sensor = _sensor_getters[sensor_class_name](**sensor_def)
    except TypeError as err:
        raise SensorSpecificationError(f'The {sensor_class_name} sensor definition is missing one or more fields. '
                                       f'These fields were defined:\n{sensor_def}\n'
                                       f'Original error message:\n{err}')
    return sensor


def get_sensors_automation(sensor_type, all_sensor_defs, bng=None, vehicle=None, name=None, dyn_sensor_properties=None):
    """
    Retrieves a sensor for automation purposes.

    Args:
        sensor_type (str): Type of the sensor (e.g., 'camera').
        all_sensor_defs (dict): Dictionary containing definitions for all sensors.
        bng (BeamNGpy, optional): BeamNGpy instance for communication.
        vehicle (Vehicle, optional): The vehicle to which the sensor is attached.
        name (str, optional): The name of the sensor.
        dyn_sensor_properties (dict, optional): Attributes that vary per sensor model, such as position and rotation.

    Returns:
        tuple: A tuple containing the initialized sensor instance and its publisher.

    Raises:
        SensorSpecificationError: If the sensor definition is missing or contains incorrect fields.
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
            sensor = _sensor_getters[sensor_class_name](bng=bng, name=name, vehicle=vehicle, **sensor_def)
            sensor_publisher = _sensor_automation_type_publisher_getter[sensor_class_name]
        else:
            sensor = _sensor_getters[sensor_class_name](**sensor_def)
            sensor_publisher = None
    except TypeError as err:
        raise SensorSpecificationError(f'The {sensor_class_name} sensor definition is missing one or more fields. '
                                       f'These fields were defined:\n{sensor_def}\n'
                                       f'Original error message:\n{err}')
    return sensor, sensor_publisher


# List of sensors that support automation
_automation_sensors = [
    'Camera',
    'Lidar',
    'AdvancedIMU',  
    'GPS',  
    'Ultrasonic',
    'IdealRadar',
    'Radar', 
    'RoadsSensor', 
    'PowertrainSensor',  
    'Mesh'
]

# Mapping of sensor type to publisher
_sensor_automation_type_publisher_getter = {
    'Lidar': LidarPublisher,
    'Camera': CameraPublisher,
    'Ultrasonic': UltrasonicPublisher,
    'AdvancedIMU': AdvancedIMUPublisher,
    'GPS': GPSPublisher,
    'IdealRadar': IdealRadarPublisher,
    'Radar': RadarPublisher,
    'RoadsSensor': RoadsSensorPublisher,
    'PowertrainSensor': PowertrainSensorPublisher,
    'Mesh': MeshPublisher
}

# Mapping of sensor types to functions that instantiate them
_sensor_getters = {
    'Damage': bng_sensors.Damage,
    'Timer': bng_sensors.Timer,
    'GForces': bng_sensors.GForces,
    'Electrics': bng_sensors.Electrics,
    'Lidar': get_lidar,
    'Camera': get_camera,
    'Ultrasonic': get_ultrasonic,
    'AdvancedIMU': get_advanced_imu,
    'GPS': get_gps,
    'IdealRadar': get_ideal_radar,
    'Radar': get_radar,
    'RoadsSensor': get_roads_sensor,
    'PowertrainSensor': get_power_train,
    'Mesh': get_mesh
    # 'IMU': bng_sensors.IMU,
    # 'CameraNoise': get_camera_noise_sensor,
    # 'LidarNoise': get_lidar_noise_sensor
}
