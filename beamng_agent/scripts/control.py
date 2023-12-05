#!/usr/bin/env python3

import sys
import rospy
import json

import beamngpy as bngpy
import beamng_msgs.msg as bng_msgs

from pathlib import Path
from distutils.version import LooseVersion

MIN_BNG_VERSION_REQUIRED = '0.18.0'
NODE_NAME = 'beamng_agent'

class VehicleControl(object):

    def __init__(self, vehicle_id):
        # params = rospy.get_param("beamng")       
        host = rospy.get_param("~host", default=None)
        port = rospy.get_param("~port", default=None)
        
        self.game_client = bngpy.BeamNGpy(host, port)
        # self.game_client = bngpy.BeamNGpy("192.168.1.145", 64256)
        # self.game_client = bngpy.BeamNGpy(params['host'], params['port'], remote=True)

        try:
            # self.game_client.open(launch=False, deploy=False)
            # self.game_client.open(listen_ip='*')
            self.game_client.open(listen_ip='*',launch=False, deploy=False)
            rospy.loginfo("Successfully connected to BeamNG.tech.")
        except TimeoutError:
            rospy.logerr("Could not establish game connection, check whether BeamNG.tech is running.")
            sys.exit(1)

        current_vehicles = self.game_client.get_current_vehicles()

        ass_msg = f"no vehicle with id {vehicle_id} exists"
        assert vehicle_id in current_vehicles.keys(), ass_msg
        self.vehicle_client = current_vehicles[vehicle_id]
        try:
            self.vehicle_client.connect(self.game_client)
            vid = self.vehicle_client.vid
            rospy.loginfo(f'Successfully connected to vehicle client with id {vid}')
        except TimeoutError:
            rospy.logfatal("Could not establish vehicle connection, system exit.")
            sys.exit(1)

        control_topic = 'control'
        rospy.Subscriber(control_topic, bng_msgs.VehicleControl, lambda x: self.send_control_signal(x))
        # rospy.logdebug(f'subscribing to "{control_topic}" for vehicle control')
        rospy.on_shutdown(lambda: self.on_shutdown())

    def on_shutdown(self):
        self.vehicle_client.disconnect()
        self.game_client.disconnect()
        node_name = rospy.get_name()
        rospy.logdebug(f'Shutting down node "{node_name}"')

    def send_control_signal(self, signal):
        self.vehicle_client.control(steering=signal.steering,
                                    throttle=signal.throttle,
                                    brake=signal.brake,
                                    parkingbrake=signal.parkingbrake,
                                    clutch=signal.clutch,
                                    gear=signal.gear
                                    )


def main():
    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.DEBUG)
    node_name = rospy.get_name()
    rospy.logdebug(f'started node "{node_name}"')

    available_version = bngpy.__version__
    if LooseVersion(MIN_BNG_VERSION_REQUIRED) > LooseVersion(available_version):
        rospy.logfatal(f'This package requires the beamngpy lib to have version {MIN_BNG_VERSION_REQUIRED}, but the available version is {available_version}, aborting process.')
        sys.exit(1)

    argv = rospy.myargv(argv=sys.argv)
    if len(argv)==2:
        VehicleControl(argv[1])
    else:
        rospy.logerr("No Vehicle ID given, shutting down node.")

    rospy.spin()

if __name__ == "__main__":
    main()
