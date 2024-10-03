#!/usr/bin/env python3


import sys
import json
import copy
from pathlib import Path
from distutils.version import LooseVersion


import numpy as np
import rospy
import rospkg
import actionlib
import tf
import tf2_ros
from tf.transformations import quaternion_from_euler, quaternion_multiply
import geometry_msgs.msg

import beamngpy as bngpy

# ROS-BeamNGpy custome message 
import beamng_msgs.msg as bng_msgs
import beamng_msgs.srv as bng_srv
from beamng_control.publishers import VehiclePublisher, NetworkPublisher, get_sensor_publisher #, NetworkPublisherR, NetworkPublisherL, NetworkPublisherM
from beamng_control.sensorHelper import get_sensors_classical, get_sensors_automation




MIN_BNG_VERSION_REQUIRED = '0.32.0'
NODE_NAME = 'beamng_control'


def load_json(file_name):
    pkg_path = rospkg.RosPack().get_path(NODE_NAME)
    file_path = Path(file_name).resolve()
    relative_fp = Path(str(pkg_path)+'/'+str(file_path))
    if not file_path.is_file() and relative_fp.is_file():
        file_path = relative_fp
    with file_path.open('r') as fh:
        content = json.load(fh)
    return content


class BeamNGBridge(object):

    def __init__(self, host, port, sensor_paths=None):
        self.game_client = bngpy.BeamNGpy(host, port)
        try:
            self.game_client.open(listen_ip='*',launch=False, deploy=False)
            rospy.loginfo("Successfully connected to BeamNG.tech")
        except TimeoutError:
            rospy.logfatal("Could not establish connection, "
                           "check whether BeamNG.tech is running.")
            sys.exit(1)

        self.running = False

        self._setup_services()
        self._publishers = list()
        self._vehicle_publisher = None
        self._static_tf_frames: list = []
        self._static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._setup_sensor_defs(sensor_paths)

        self._stepAS = actionlib.SimpleActionServer(f'{NODE_NAME}/step',
                                                    bng_msgs.StepAction,
                                                    execute_cb=self.step,
                                                    auto_start=False)
        rospy.on_shutdown(lambda: self.on_shutdown())

        self._stepAS.start()
        self._marker_idx = 0
        self.network_publisher = None

    def get_bng(self):
        bng =  self.bng
        return bng

    def _setup_sensor_defs(self, sensor_paths):
        default_path = ['/config/sensors.json']
        sensor_paths = default_path if not sensor_paths else sensor_paths
        print("------------")
        print("------------")
        print ("sensor_paths",sensor_paths)
        print("------------")
        print("------------")
        sensor_defs = dict()
        for path in sensor_paths:
            s = load_json(path)
            sensor_defs.update(s)
        self._sensor_defs = sensor_defs

    def _setup_services(self):
        self.add_service('get_scenario_state',
                         bng_srv.GetScenarioState,
                         self.get_scenario_state)
        self.add_service('spawn_vehicle',
                         bng_srv.SpawnVehicle,
                         self.spawn_new_vehicle)
        self.add_service('start_scenario',
                         bng_srv.StartScenario,
                         self.start_scenario_from_req)
        self.add_service('get_current_vehicles',
                         bng_srv.GetCurrentVehiclesInfo,
                         self.get_current_vehicles)
        self.add_service('teleport_vehicle',
                         bng_srv.TeleportVehicle,
                         self.teleport_vehicle)
        self.add_service('pause',
                         bng_srv.ChangeSimulationState,
                         self.pause)
        self.add_service('resume',
                         bng_srv.ChangeSimulationState,
                         self.resume)

    def add_service(self, service_name, srv_type, func):
        service_name = f'{NODE_NAME}/{service_name}'
        rospy.Service(service_name, srv_type, func)
        rospy.loginfo(f'Added service "{service_name}".')

    def get_marker_idx(self):
        m = self._marker_idx
        self._marker_idx += 1
        return m




    def get_sensor_classical_from_dict(self, v_spec):
        vehicle = bngpy.Vehicle(v_spec['name'], v_spec['model'])
        sensor_collection = list()
        noise_sensors = list()
        if 'sensors_classical' in v_spec:
            for spec in v_spec['sensors_classical']:
                if 'base sensor' in spec:
                    noise_sensors.append(spec)
                else:
                    sensor_collection.append(spec)
        rospy.logdebug(f'sensors_classical: {sensor_collection}')
        for s_spec in sensor_collection:
            s_name = s_spec.pop('name')
            s_type = s_spec.pop('type')
            # rospy.logdebug(f'Attempting to set up {s_type} sensor.')
            sensor = get_sensors_classical(s_type,
                                self._sensor_defs,
                                dyn_sensor_properties=s_spec)
            vehicle.attach_sensor(s_name, sensor)

        return vehicle
    

    
    


    @staticmethod
    def get_stamped_static_tf_frame(translation, rotation, vehicle_name: str, sensor_name: str):
        static_transform_stamped = geometry_msgs.msg.TransformStamped()
        static_transform_stamped.header.frame_id = vehicle_name
        static_transform_stamped.child_frame_id = f"{vehicle_name}_{sensor_name}"

        static_transform_stamped.transform.translation.x = float(translation[0])
        static_transform_stamped.transform.translation.y = float(translation[1])
        static_transform_stamped.transform.translation.z = float(translation[2])

        quat = tf.transformations.quaternion_from_euler(float(0),
                                                        float(rotation[0]),
                                                        float(rotation[1]))  # RPY to convert

        alignment_quat = np.array([0, 0, 0, 1])  # sets the forward direction as -y
        # alignment_quat = np.array([0, 1, 0, 0])  # sets the forward direction as -y
        quat = quaternion_multiply(alignment_quat, quat)
        quat /= np.linalg.norm(quat)
        static_transform_stamped.transform.rotation.x = quat[0]
        static_transform_stamped.transform.rotation.y = quat[1]
        static_transform_stamped.transform.rotation.z = quat[2]
        static_transform_stamped.transform.rotation.w = quat[3]
        return static_transform_stamped

    def set_sensors_automation_from_dict(self, scenario_spec, vehicle_list):
        for v_spec, vehicle in zip(scenario_spec['vehicles'], vehicle_list):
            sensor_collection = list()
            noise_sensors = list()
            if 'sensors_automation' in v_spec:
                for spec in v_spec['sensors_automation']:
                    if 'base sensor' in spec:
                        noise_sensors.append(spec)
                    else:
                        sensor_collection.append(spec)
            rospy.logdebug(f'sensors_automation: {sensor_collection}')
            rospy.logdebug(f'noise_sensors_automation: {noise_sensors}')
            for s_spec in sensor_collection:
                dyn_spec = copy.deepcopy(s_spec)
                dyn_spec.pop("name")
                dyn_spec.pop("type")
                s_type = s_spec["type"]
                name = s_spec["name"]

                rospy.logdebug(f'Attempting to set up {s_type} sensor.')
                # send information of sensors to SensorHelper.py then from there to publishers.py
                sensor, sensor_publisher = get_sensors_automation(s_type,
                                                      self._sensor_defs,
                                                      bng=self.game_client,
                                                      vehicle=vehicle,
                                                      name=name,
                                                      dyn_sensor_properties=dyn_spec)
                rospy.logdebug(f' sensor  {sensor} ')                
                rospy.logdebug(f' sensor_publisher  {sensor_publisher} ')
                rospy.logdebug(f' vehicle.vid {vehicle.vid} ')
                
                if sensor_publisher is not None:
                    static_sensor_frame = self.get_stamped_static_tf_frame(translation=s_spec['position'],
                                                                           rotation=s_spec['rotation'],
                                                                           vehicle_name=vehicle.vid,
                                                                           sensor_name=name)
                    self._static_tf_frames.append(static_sensor_frame)
                    # rospy.logdebug(f' static_sensor_frame  {static_sensor_frame} ')                
                    self._publishers.append(sensor_publisher(sensor, f"{NODE_NAME}/{vehicle.vid}/{name}", vehicle))

    @staticmethod
    def get_vehicle_from_dict(v_spec):
        vehicle = bngpy.Vehicle(v_spec['name'], v_spec['model'])
        return vehicle
   

    def _scenario_from_json(self, file_name):
        try:
            scenario_spec = load_json(file_name)
        except FileNotFoundError:
            rospy.logerr(f'file "{file_name}" does not exist, abort')
            return
        rospy.loginfo(json.dumps(scenario_spec))
        return scenario_spec

    def decode_scenario(self, scenario_spec):
        vehicle_list = list()
        scenario = bngpy.Scenario(scenario_spec.pop('level'),
                                  scenario_spec.pop('name'))
        for v_spec in scenario_spec['vehicles']:
            # NOT USED IN NEW CODE
            vehicle = self.get_sensor_classical_from_dict(v_spec)

            # self._publishers.append(VehiclePublisher(vehicle, NODE_NAME))  # todo markers need to be added somwhere else
            
            # USED IN NEW CODE 
            # vehicle = self.get_sensor_classical_from_dict(v_spec, vehicle)
            # send the vehicle information with classical sensors to publishers.py 
            self._vehicle_publisher = VehiclePublisher(vehicle, NODE_NAME)  # we need this to be published first for tf
            vehicle_list.append(vehicle)
            scenario.add_vehicle(vehicle, 
                                 pos=v_spec['position'],
                                 rot_quat=v_spec['rotation'])
            rospy.logdebug(f'vehicle in decode_scenario: {vehicle}')



        on_scenario_start = list()
        wp_key = 'weather_presets'
        if wp_key in scenario_spec.keys():
            def weather_presets():
                self.game_client.set_weather_preset(scenario_spec[wp_key])
            on_scenario_start.append(weather_presets)
        if 'time_of_day' in scenario_spec.keys():
            def tod():
                self.game_client.set_tod(scenario_spec['time_of_day'])
            on_scenario_start.append(tod)
        net_viz_key = 'network_vizualization'
        if net_viz_key in scenario_spec and scenario_spec[net_viz_key] == 'on':
            self._publishers.append(NetworkPublisher(self.game_client, NODE_NAME))
            # self._publishers.append(NetworkPublisherR(self.game_client, NODE_NAME))
            # self._publishers.append(NetworkPublisherL(self.game_client, NODE_NAME))
            # self._publishers.append(NetworkPublisherM(self.game_client, NODE_NAME))
        return scenario, on_scenario_start, vehicle_list



    def start_scenario(self, file_name):
        self._publishers = list()
        scenario_spec = self._scenario_from_json(file_name)
        if not scenario_spec:
            return
        scenario, on_scenario_start, vehicle_list = self.decode_scenario(scenario_spec)
        scenario.make(self.game_client)
        self.game_client.load_scenario(scenario)
        self.game_client.start_scenario()
        self.set_sensors_automation_from_dict(scenario_spec, vehicle_list)

        
        for hook in on_scenario_start:
            hook()

        if 'mode' in scenario_spec.keys() and scenario_spec['mode'] == 'paused':
            rospy.logdebug("paused scenario")
            self.game_client.pause()
        else:
            rospy.logdebug("non paused scenario")
        rospy.loginfo(f'Started scenario "{scenario.name}".')
        self.running = True


    def get_scenario_state(self, req):
        rospy.loginfo(f'get_scenario_state is called.')
        response = bng_srv.GetScenarioStateResponse()
        response.state.loaded = False
        response.state.running = False
        response.state.scenario_name = ""
        response.state.level_name = ""
        game_state = self.game_client.get_gamestate()
        response.state.vehicle_ids = []
        rospy.loginfo(f'game_state "{game_state}".')        
        try:
            if game_state['state'] == 'scenario':
                response.state.loaded = True
                response.state.level_name = game_state['level']
                vehicles = self.game_client.get_current_vehicles()
                vehicles = list(vehicles.keys())
                response.state.vehicle_ids = vehicles
                rospy.loginfo(f'game_state scenario "{response}".')        
                if 'scenario_state' in game_state.keys():
                    if game_state['scenario_state'] == 'running':
                        response.state.running = True
                    response.state.scenario_name = self.game_client.get_scenario_name()
                    rospy.loginfo(f'game_state 1st if "{response}".')        
            elif game_state['state'] == 'menu':
                rospy.loginfo(f'from elif.')
                # response.state.loaded = True
                # vehicles = self.game_client.get_current_vehicles()
                # vehicles = list(vehicles.keys())
                # response.state.vehicle_ids = vehicles
                # response.state.scenario_name = self.game_client.get_scenario_name()
                response.state.scenario_name=game_state['state']
                rospy.loginfo(f'game_state "{response}".')        
            else :
                rospy.loginfo(f'game_state "{game_state}".') 
        except Exception as e:
            rospy.logerr(f'An error occurred: {str(e)}')
            return response  
        return response  
    

    def spawn_new_vehicle(self, req):
        rospy.logdebug(f'spawn vehicle req: {str(req)}')
        response = bng_srv.SpawnVehicleResponse()
        response.success = False
        try:
            vehicle_spec = load_json(req.path_to_vehicle_config_file)
        except FileNotFoundError:
            rospy.logerr(f'file "{req.path_to_vehicle_config_file}" '
                         'does not exist, aborting subroutine')
            return response
        if len(req.pos) != 3:
            rospy.logerr('position param does not fit '
                         f'required format:{str(req.pos)}')
            return response
        if len(req.rot_quat) != 4:
            rospy.logerr('rotation param does not fit '
                         'required quaternion format:{str(req.rot_quat)}')
            return response
        vehicle_spec['name'] = req.name
        vehicle = self.get_vehicle_from_dict(vehicle_spec)#wip
        self.game_client.spawn_vehicle(vehicle,
                                       req.pos,
                                       rot_quat=req.rot_quat)
        

        
        response.success = True
        return response


    def start_scenario_from_req(self, req):
        self.start_scenario(req.path_to_scenario_definition)
        response = bng_srv.StartScenarioResponse()
        response.success = True
        return response
    
    def get_current_vehicles(self, req):
        rospy.loginfo(f'get_current_vehicles is called .')
        response = bng_srv.GetCurrentVehiclesInfoResponse()
        vehicles = list()
        list_of_current_vehicles = self.game_client.get_current_vehicles_info()
        rospy.loginfo(f'Started scenario "{list_of_current_vehicles}".')
        for veh in list_of_current_vehicles.values():
            veh_inf = bng_msgs.VehicleInfo()
            veh_inf.vehicle_id = veh['name']
            veh_inf.model = veh['model']
            vehicles.append(veh_inf)
        response.vehicles = vehicles
        rospy.loginfo(f'vehicles "{vehicles}".')
        response=True
        return response
 


    def teleport_vehicle(self, req):
        response = bng_srv.TeleportVehicleResponse()
        response.success = False
        if len(req.pos) != 3:
            rospy.logerr('position param does not fit '
                         f'required format:{str(req.pos)}')
            return response
        if len(req.rot_quat) != 4:
            rospy.logerr('rotation param does not fit '
                         f'required quaternion format:{str(req.rot_quat)}')
            return response
        success = self.game_client.teleport_vehicle(req.vehicle_id,
                                                    req.pos,
                                                    rot_quat=req.rot_quat)
        if success:
            response.success = True
        return response
    

    def pause(self, req):
        response = bng_srv.ChangeSimulationStateResponse()
        try:
            self.game_client.pause()
        except bngpy.beamngcommon.BNGError:
            rospy.logerr('No confirmation available, '
                         'simulation may or may not have paused.')
            response.success = False
            return response
        response.success = True
        return response

    def resume(self, req):
        response = bng_srv.ChangeSimulationStateResponse()
        try:
            self.game_client.resume()
        except bngpy.beamngcommon.BNGError:
            rospy.logerr('No confirmation available, '
                         'simulation may or may not have resumed.')
            response.success = False
            return response
        response.success = True
        return response

    def step(self, goal):
        success = True

        feedback = bng_msgs.StepFeedback()

        step_counter = 0
        imax = goal.total_number_of_steps//goal.feedback_cycle_size
        rest = goal.total_number_of_steps % goal.feedback_cycle_size
        imax = imax+1 if rest else imax

        for i in range(0, imax):
            if self._stepAS.is_preempt_requested():
                self._stepAS.set_preempted()
                success = False
                rospy.loginfo("Step action preempted: completed "
                              f"{step_counter}/{goal.total_number_of_steps} "
                              "steps")
                break
            steps_to_finish = goal.total_number_of_steps-step_counter
            step_size = min(steps_to_finish, goal.feedback_cycle_size)
            self.game_client.step(step_size)
            step_counter += step_size
            # rospy.logdebug(f"took {step_counter}/"
            #                f"{goal.total_number_of_steps} steps")
            feedback.steps_completed = step_counter
            self._stepAS.publish_feedback(feedback)

        if success:
            rospy.loginfo(f"completed goal, performed {step_counter} steps")
            result = bng_msgs.StepResult()
            result.success = True
            self._stepAS.set_succeeded(result)

    def get_roads(self):
        roads = self.game_client.get_roads()
        road_spec = {}
        for r_id, r_inf in roads.items():
            if r_inf['drivability'] != '-1':
                road_spec[r_id] = self.game_client.get_road_edges(r_id)

        network = list()
        for r_id in road_spec.keys():
            left = list()
            right = list()
            for r_point in road_spec[r_id]:
                left.append(r_point['left'])
                right.append(r_point['right'])
            if left:
                network.append(left)
            if right:
                network.append(right)
        return network

    def work(self):
        ros_rate = 10
        rate = rospy.Rate(ros_rate)  # todo add threading
        if self.running:
            while not rospy.is_shutdown():
                current_time = rospy.Time.now()
                for static_tf in self._static_tf_frames:
                    static_tf.header.stamp = current_time
                    self._static_tf_broadcaster.sendTransform(static_tf)
                if self._vehicle_publisher is not None:
                    self._vehicle_publisher.publish(current_time)
                for pub in self._publishers:
                    pub.publish(current_time)
                rate.sleep()


    def on_shutdown(self):
        rospy.loginfo("Shutting down beamng_control/bridge.py node")
        self.game_client.disconnect()


def main():
    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.DEBUG)
    node_name = rospy.get_name()
    rospy.loginfo(f'Started node "{node_name}".')

    available_version = bngpy.__version__
    if LooseVersion(MIN_BNG_VERSION_REQUIRED) > LooseVersion(available_version):
        rospy.logfatal('This package requires at least BeamNGpy '
                       f'version {MIN_BNG_VERSION_REQUIRED}, but the available'
                       f' version is {available_version}, aborting process.')
        sys.exit(1)


    # bngpy.setup_logging()
    argv = rospy.myargv(argv=sys.argv)
    rospy.loginfo("cmd args"+str(argv))

    host = rospy.get_param("~host", default=None)
    port = rospy.get_param("~port", default=None)
    if host is None or port is None:
        rospy.logfatal("No host or port specified on the parameter server to connect to Beamng.tech")
        sys.exit()
    bridge = BeamNGBridge(host, port)
    if len(argv) == 2:
        rospy.logdebug('Detected optional input, '
                       f'creating scenario from json: "{argv[1]}"')
        scenario_print=argv[1]
        print("scenario_print",scenario_print)
        bridge.start_scenario(argv[1])
        
    bridge.work()


if __name__ == "__main__":
    main()
