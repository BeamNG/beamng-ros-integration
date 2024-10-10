#!/usr/bin/env python3

import sys
import rospy
import json

import beamngpy as bngpy
import beamng_msgs.msg as bng_msgs
from beamng_msgs.srv import AiControl

from pathlib import Path
from distutils.version import LooseVersion

# Minimum required version of BeamNGpy for this node
MIN_BNG_VERSION_REQUIRED = '0.31.0'
# ROS node name
NODE_NAME = 'beamng_agent'

class VehicleControl(object):
    """
    A class that interfaces with a specific vehicle in BeamNG.tech and provides
    control over its AI and driving commands.
    """

    def __init__(self, vehicle_id):
        """
        Initializes the vehicle control object and establishes a connection to the
        BeamNG.tech simulation environment and the specified vehicle.

        Args:
            vehicle_id (str): The ID of the vehicle to connect to.
        """
        # Retrieve parameters for BeamNG host, port, and driving mode from ROS parameters
        host = rospy.get_param("~host", default=None)
        port = rospy.get_param("~port", default=None)
        driving_mode = rospy.get_param("~driving_mode", default="ai")
        
        # Initialize the BeamNGpy client with the given host and port
        self.game_client = bngpy.BeamNGpy(host, port)

        try:
            # Attempt to connect to BeamNG.tech
            self.game_client.open(listen_ip='*', launch=False, deploy=False)
            rospy.loginfo("Successfully connected to BeamNG.tech.")
        except TimeoutError:
            rospy.logerr("Could not establish game connection, check whether BeamNG.tech is running.")
            sys.exit(1)

        # Get the current vehicles in the simulation
        current_vehicles = self.game_client.get_current_vehicles()

        # Ensure the vehicle ID exists in the current simulation
        ass_msg = f"No vehicle with ID {vehicle_id} exists"
        assert vehicle_id in current_vehicles.keys(), ass_msg
        self.vehicle_client = current_vehicles[vehicle_id]

        try:
            # Connect to the specific vehicle
            self.vehicle_client.connect(self.game_client)
            vid = self.vehicle_client.vid
            rospy.loginfo(f'Successfully connected to vehicle client with ID {vid}')
        except TimeoutError:
            rospy.logfatal("Could not establish vehicle connection, system exiting.")
            sys.exit(1)

        # Subscribe to the /control topic to receive control signals for the vehicle
        rospy.Subscriber('/control', bng_msgs.VehicleControl, lambda x: self.send_control_signal(x, driving_mode))
        # Register a shutdown hook to cleanly disconnect from the simulation
        rospy.on_shutdown(lambda: self.on_shutdown())

        # Create a ROS service for AI control
        self.ai_control_service = rospy.Service('ai_control', AiControl, self.ai_control)

    def on_shutdown(self):
        """
        Disconnects the vehicle and game client from the BeamNG.tech simulation.
        This function is called when the node is shutting down.
        """
        self.vehicle_client.disconnect()
        self.game_client.disconnect()
        node_name = rospy.get_name()
        rospy.logdebug(f'Shutting down node "{node_name}"')

    def send_control_signal(self, signal, mode):
        """
        Sends control signals to the vehicle. Depending on the mode, it either
        sets the AI mode or sends manual control commands.

        Args:
            signal (VehicleControl): The control signal message containing steering, throttle, etc.
            mode (str): The driving mode (either 'ai' or manual).
        """
        if mode == "ai":
            # Set AI to follow a span path
            self.vehicle_client.ai_set_mode('span')
        else:
            # Send manual control commands to the vehicle
            self.vehicle_client.control(steering=signal.steering,
                                        throttle=signal.throttle,
                                        brake=signal.brake,
                                        parkingbrake=signal.parkingbrake,
                                        clutch=signal.clutch,
                                        gear=signal.gear)

    def teleport_vehicle(self, req):
        """
        Teleports the vehicle to a specified position and orientation.

        Args:
            req: A service request containing the vehicle ID, position, and rotation quaternion.

        Returns:
            A service response indicating whether the teleportation was successful.
        """
        response = bng_srv.TeleportVehicleResponse()
        response.success = False

        # Validate position and rotation parameters
        if len(req.pos) != 3:
            rospy.logerr('Position parameter does not fit the required format: {str(req.pos)}')
            return response
        if len(req.rot_quat) != 4:
            rospy.logerr('Rotation quaternion parameter does not fit the required format: {str(req.rot_quat)}')
            return response
        
        # Teleport the vehicle
        success = self.game_client.teleport_vehicle(req.vehicle_id, req.pos, rot_quat=req.rot_quat)
        if success:
            response.success = True
        return response

    def ai_control(self, req):
        """
        Enables or disables AI control for the vehicle based on the service request.

        Args:
            req: A service request containing a boolean to enable or disable AI control.

        Returns:
            AiControl: A service response indicating whether the AI control was successful.
        """
        response = AiControl()
        try:
            if req.enable:
                rospy.loginfo("Enabling AI control...")
                self.vehicle_client.ai_set_mode('span')
            else:
                rospy.loginfo("Disabling AI control...")
                self.vehicle_client.ai_set_mode('disable')
        except Exception as e:
            rospy.logerr(f"Failed to set AI mode: {e}")

def main():
    """
    The main entry point for the ROS node. Initializes the node, checks for
    version compatibility, and starts the vehicle control.
    """
    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.DEBUG)
    node_name = rospy.get_name()
    rospy.logdebug(f'Started node "{node_name}"')

    # Check if the current version of BeamNGpy meets the minimum requirement
    available_version = bngpy.__version__
    if LooseVersion(MIN_BNG_VERSION_REQUIRED) > LooseVersion(available_version):
        rospy.logfatal(f'This package requires BeamNGpy lib version {MIN_BNG_VERSION_REQUIRED}, but the available version is {available_version}, aborting process.')
        sys.exit(1)

    # Process command-line arguments for vehicle ID
    argv = rospy.myargv(argv=sys.argv)
    if len(argv) == 2:
        VehicleControl(argv[1])
    else:
        rospy.logerr("No Vehicle ID given, shutting down node.")

    rospy.spin()

if __name__ == "__main__":
    main()
