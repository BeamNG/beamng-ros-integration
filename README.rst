BeamNG ROS Integration
^^^^^^^^^^^^^^^^^^^^^^^

About
=============

This repository contains packages to support the interoperability between BeamNG.tech and ROS.
BeamNG.tech is a driving simulation platform, suitable for commercial and academic use.
Free licenses are available for non-commercial and academic use.
Inquiries can be made through our [registration form](https://register.beamng.tech/).
For inquiries regarding commercial use, contact us at <licensing@beamng.com>.

Features
=============
As of now the BeamNG ROS integration supports one package for the **remote** control of the simulation platform and one package for the control of a driving agent. A third package manages custom messages.

Prerequirements
=============

For using the BeamNG ROS integration, a BeamNG.tech build and a python environment with [BeamNGpy][1] installed are required.

Note that BeamNG.tech **only runs on Window**, although Linux support is on its way.
That means that BeamNG.tech needs to run on a separate Windows machine, or that ROS needs to run on on WSL.

The BeamNG ROS integration is compatible with the ROS 1 distributions Melodic Morenia and  Noetic Ninjemys.

Getting Started
=============

To use this project, a basic knowledge of the BeamNG.tech simulator and the BeamNGpy is neccessary. We recommend to familiarize yourself first with [BeamNGpy][1] to get a basic understanding of the platform before using the BeamNG ROS Integration.

After setting up BeamNG.tech and BeamNGpy with a python environment, the simulation needs to be started through BeamNGpy.

The ROS packages from this repository need to be added and build in your catkin workspace.
See the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials) for more information on how to set up a catkin workspace.

A node connecting ROS to the simulation can then be started with the help of the `example.launch` file in the `beamng_control` package through the command:

```shell
roslaunch beamng_control example.launch
```

It needs to be configured to contain the correct IP address of the machine hosting the simulation.
Using it will start up a node that connects to the simulation and starts up a scenario as defined in the `beamng_control/config/simple_scenario.json`.
Other scenario specifications are available in the same directory.

Teleop control
=============

[beamng_teleop_keyboard](https://github.com/BeamNG/beamng-ros-integration/tree/master/beamng_teleop_keyboard) is a generic Keyboard Packages is built for teleoperating ROS robots using Twist message from [geometry_messages](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html). 
 
#### Running beamng_teleop_keyboard: 
- Loading BeamNG-ROS bridge:
```
roslaunch beamng_control example.launch
```
 
- Calling Twist_message converter node:
```
rosrun beamng_teleop_keyboard converter
```
 
- calling Teleop node:
```
rosrun beamng_teleop_keyboard teleop_key
```
 
- Loading beamng_agent node:
```
roslaunch beamng_agent example.launch 
```


Compatibility
=============

Running the BeamNG ROS integration requires three individual software components, here is a list of compatible versions.

| BeamNG.tech | BeamNGpy | BeamNG ROS Integration |
|-------------|----------|------------------------|
| 0.25|1.23| 0.1 |

[1]: https://github.com/BeamNG/BeamNGpy
