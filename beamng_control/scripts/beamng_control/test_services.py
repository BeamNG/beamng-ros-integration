#!/usr/bin/env python3

import unittest
import sys
import time

import numpy as np

import rospy
import rospkg
import rostest
import beamng_msgs.srv as bng_srv
import beamng_msgs.msg as bng_msg

SRV_PREFIX = '/beamng_control/'
PKG = 'beamng_control'
NAME = 'bidge_services_test'


def start_scenario(scenario_name):
    service_name = SRV_PREFIX + 'start_scenario'
    rospy.wait_for_service(service_name)
    client = rospy.ServiceProxy(service_name, bng_srv.StartScenario)
    req = bng_srv.StartScenarioRequest(scenario_name)
    resp = client.call(req)
    return resp.success


class TestBridgeServices(unittest.TestCase):

    def test_start_scenario(self):
        self.assertTrue(start_scenario('/test/scenarios/smallgrid.json'))

    def test_get_scenario_state(self):
        service_name = SRV_PREFIX + 'get_scenario_state'
        rospy.wait_for_service(service_name)
        client = rospy.ServiceProxy(service_name, bng_srv.GetScenarioState)
        req = bng_srv.GetScenarioStateRequest()
        resp = client.call(req)

        self.assertTrue(start_scenario('/test/scenarios/smallgrid.json'))

        resp = client.call(req)
        self.assertTrue(resp.state.running)
        self.assertTrue(resp.state.loaded)

    def test_spawn_vehicle(self):

        service_name = SRV_PREFIX + 'spawn_vehicle'
        rospy.wait_for_service(service_name)
        client = rospy.ServiceProxy(service_name, bng_srv.SpawnVehicle)

        self.assertTrue(start_scenario('/test/scenarios/smallgrid.json'))

        vehicle_path = "/test/vehicles/etk800.json"
        req = bng_srv.SpawnVehicleRequest('ut_1',
                                          (5, 0, 0),
                                          (0, 0, 0, 1),
                                          vehicle_path)
        resp = client.call(req)
        self.assertTrue(resp.success)

        vehicle_path = rospkg.RosPack().get_path('beamng_control') + "/test/vehicles/pickup.json"
        req = bng_srv.SpawnVehicleRequest('ut_2',
                                          (10, 0, 0),
                                          (0, 0, 0, 1),
                                          vehicle_path)
        resp = client.call(req)
        self.assertTrue(resp.success)

    def test_teleport_vehicle(self):
        self.assertTrue(start_scenario('/test/scenarios/smallgrid.json'))
        service_name = SRV_PREFIX + 'teleport_vehicle'
        vehicle_name = 'ego_vehicle'
        rospy.wait_for_service(service_name)
        client = rospy.ServiceProxy(service_name,
                                    bng_srv.TeleportVehicle)
        req = bng_srv.TeleportVehicleRequest(vehicle_name,
                                             (10, 10, 0),
                                             (0, 0, 0, 1))
        resp = client.call(req)
        self.assertTrue(resp.success)

    def test_pause(self):
        self.assertTrue(start_scenario('/test/scenarios/smallgrid.json'))
        service_name = SRV_PREFIX + 'pause'
        rospy.wait_for_service(service_name)
        client = rospy.ServiceProxy(service_name,
                                    bng_srv.ChangeSimulationState)
        req = bng_srv.ChangeSimulationStateRequest()
        resp = client.call(req)
        self.assertTrue(resp.success)

    def test_resume(self):
        self.assertTrue(start_scenario('/test/scenarios/smallgrid.json'))
        service_name = SRV_PREFIX + 'resume'
        rospy.wait_for_service(service_name)
        client = rospy.ServiceProxy(service_name,
                                    bng_srv.ChangeSimulationState)
        req = bng_srv.ChangeSimulationStateRequest()
        resp = client.call(req)
        self.assertTrue(resp.success)

    def test_get_current_vehicles(self):
        self.assertTrue(start_scenario('/test/scenarios/smallgrid.json'))
        service_name = SRV_PREFIX + 'get_current_vehicles'
        rospy.wait_for_service(service_name)
        client = rospy.ServiceProxy(service_name,
                                    bng_srv.GetCurrentVehiclesInfo)
        req = bng_srv.GetCurrentVehiclesInfoRequest()
        resp = client.call(req)
        self.assertTrue(len(resp.vehicles) == 1)
        self.assertTrue(resp.vehicles[0].vehicle_id == 'ego_vehicle')
        self.assertTrue(resp.vehicles[0].model == 'etk800')


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestBridgeServices, sys.argv)