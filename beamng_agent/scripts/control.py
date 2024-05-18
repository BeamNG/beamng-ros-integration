#!/usr/bin/env python3

import sys
import rospy
import json

import beamngpy as bngpy
import beamng_msgs.msg as bng_msgs
from beamng_msgs.srv import AiControl

from pathlib import Path
from distutils.version import LooseVersion

MIN_BNG_VERSION_REQUIRED = '0.31.0'
NODE_NAME = 'beamng_agent'

class VehicleControl(object):

    def __init__(self, vehicle_id):
        # params = rospy.get_param("beamng")       
        host = rospy.get_param("~host", default=None)
        port = rospy.get_param("~port", default=None)
        driving_mode = rospy.get_param("~driving_mode", default="ai")
        
        self.game_client = bngpy.BeamNGpy(host, port)

        try:
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

        rospy.Subscriber('/control', bng_msgs.VehicleControl, lambda x: self.send_control_signal(x, driving_mode))
        rospy.on_shutdown(lambda: self.on_shutdown())

        # Create service
        self.ai_control_service = rospy.Service('ai_control', AiControl, self.ai_control)

    def on_shutdown(self):
        self.vehicle_client.disconnect()
        self.game_client.disconnect()
        node_name = rospy.get_name()
        rospy.logdebug(f'Shutting down node "{node_name}"')

    def send_control_signal(self, signal, mode):
        if mode == "ai" :
            self.vehicle_client.ai_set_mode('span')
        
        else:
            self.vehicle_client.control(steering=signal.steering,
                                    throttle=signal.throttle,
                                    brake=signal.brake,
                                    parkingbrake=signal.parkingbrake,
                                    clutch=signal.clutch,
                                    gear=signal.gear
                                    )
            

 
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
    
    
    
#rosservice call /ai_control "enable: true"
# rosservice call /ai_control "enable: false"
    def ai_control(self, req):
        response = AiControl()
        try:
            if req.enable:
                rospy.loginfo("Enabling AI control...")
                self.vehicle_client.ai_set_mode('span')
                # return True
            else:
                rospy.loginfo("Disabling AI control...")
                self.vehicle_client.ai_set_mode('disable')
                # return True
            # response.success = True
            # return True
        except Exception as e:
            rospy.logerr(f"Failed to set AI mode: {e}")
            # response.success = False
            # return False

def main():
    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.DEBUG)
    node_name = rospy.get_name()
    rospy.logdebug(f'started node "{node_name}"')

    available_version = bngpy.__version__
    if LooseVersion(MIN_BNG_VERSION_REQUIRED) > LooseVersion(available_version):
        rospy.logfatal(f'This package requires the beamngpy lib to have version {MIN_BNG_VERSION_REQUIRED}, but the available version is {available_version}, aborting process.')
        sys.exit(1)

    argv = rospy.myargv(argv=sys.argv)
    if len(argv) == 2:
        VehicleControl(argv[1])
    else:
        rospy.logerr("No Vehicle ID given, shutting down node.")

    rospy.spin()

if __name__ == "__main__":
    main()
