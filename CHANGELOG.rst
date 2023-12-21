Changelog
=========

Version 0.4 
=========================
- **LIDAR Orientation:** Corrected the orientation of the LIDAR sensor.

- **Electrics Sensors:** Fixed attributes of the Electrics sensors.

- **Ultrasonic Sensor API:** Fixed the Ultrasonic sensor API.

- **Ultrasonic Sensor Message Type:** Changed the message type of the Ultrasonic sensor.

- **BeamNGpy API Update:** Updated connection API in BeamNGpy:
    - Removed the ``remote`` parameter from ``<bngpy>.BeamNGpy``.
    - Added ``listen_ip='*'`` to ``<game_client>.open``.

- **Release Updates:** Updated release numbers from ``0.1.2`` and ``0.1.3`` to ``0.2`` and ``0.2.1,`` respectively.

- **Launch Configuration:** Removed ``bridge.launch`` and now depend only on ``example.launch.`` Minimized the arguments by removing ``ns group.``

Version 0.3 
=========================
- **Automation Sensors:** Fixed the API for the automation sensors (Camera, LIDAR, and Ultrasonic).

- **Static Transform:** Added a static transform function for the automation sensors.

- **Sensor Categories:** Added two categories for sensors: automation sensors and classical sensors.

- **Time Synchronization:** Added time synchronization between ROS-bridge and transforms in RViz.

- **LIDAR Visualization:** Added LIDAR signals to the RViz visualizer.

- **Acknowledgment:** Thanks to `@podgorki <https://github.com/podgorki>`_ for the contribution.

Version 0.2.1 
=========================
- **Code Update:** Updated the code with the latest BeamNGpy 1.26 and BeamNG.Tech 0.28.

- **Bug Fix:** Fixed the issue with handbrake and throttle.

Version 0.2 
=========================
- **Keyboard Control:** Added ``beamng_teleop_keyboard`` for keyboard control of the vehicle (from ROS).

- **New Topic:** Added ``/cmd_vel`` topic.

- **Handbrake Control:** Added handbrake (parking brake) control.

Version 0.1.1 
=========================
- **Zero Velocity:** Added zero velocity by pressing the space bar.

Version 0.1 
=========================
- **Basic Bridge:** Basic bridge with BeamNG.Tech.

- **ROS Packages:** Three ROS packages are released:
    - ``beamng_control``: To establish communication and load scenarios that can choose the vehicle and the equipped sensors.
    - ``beamng_agent``: To give the ability to control the vehicle from ROS using simple commands (move, stop, release - to allow the user to control the vehicle from BeamNG.Tech).
    - ``beamng_msgs``: A converter package to adapt BeamNGpy messages with ROS.
