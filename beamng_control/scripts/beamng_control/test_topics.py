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
NAME = 'bridge_topics_test'


def start_scenario(scenario_name):
    service_name = SRV_PREFIX + 'start_scenario'
    rospy.wait_for_service(service_name)
    client = rospy.ServiceProxy(service_name, bng_srv.StartScenario)
    req = bng_srv.StartScenarioRequest(scenario_name)
    resp = client.call(req)
    return resp.success


class TestBridgeTopics(unittest.TestCase):

    def test_start_scenario(self):
        self.assertTrue(start_scenario('/test/scenarios/test_sensors.json'))
        rospy.logerr(rospy.get_published_topics())
        vehicle_name = 'ego_vehicle'
        topics = rospy.get_published_topics()
        rospy.logerr(topics)
        published_topics = list()
        for t in topics:
            published_topics.extend(t)
        topic_prefix = f'/{PKG}/{vehicle_name}/'
        sensor_topic_suffixes = ['state', 'damage', 'time', 'gforce', 'lidar', 'lidar_noise']
        cam_suffixes = ['color', 'depth', 'instance', 'annotation']
        sensor_topic_suffixes.extend([f'front_cam/{x}' for x in cam_suffixes])
        sensor_topic_suffixes.extend([f'noise_cam/{x}' for x in cam_suffixes])
        sensor_topics = [topic_prefix + suffix for suffix in sensor_topic_suffixes]
        for t in sensor_topics:
            self.assertTrue(t in published_topics)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestBridgeTopics, sys.argv)