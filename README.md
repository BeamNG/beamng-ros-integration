# BeamNG ROS Integration

## About

This repository contains packages to support the interoperability between [BeamNG.tech](https://beamng.tech/) and ROS.
BeamNG.tech is a driving simulation platform, suitable for commercial and academic use.
Free licenses are available for non-commercial and academic use.
Inquiries can be made through our [registration form](https://register.beamng.tech/).
For inquiries regarding commercial use, contact us at <licensing@beamng.com>.
## Table of Contents


 - [Documentation](#docs)
 - [Features](#features) 
 - [Dependencies](#prereqs)
 - [Getting Started](#getstart)
 - [Compatibility](#compatibility)
 - [Troubleshooting](#troubleshooting)
 - [Contributions](#contributions)
 
<a name="docs"></a>
## Documentation
[![](https://raw.githubusercontent.com/ChristianBirchler/sdc-scissor/main/docs/images/readthedocs.png)](https://beamngpy.readthedocs.io/en/latest/bngros.html)

## Features

As of now the BeamNG ROS integration supports one package for the **remote** control of the simulation platform and one package for the control of a driving agent. A third package manages custom messages.




<a name="prereqs"></a>

## Dependencies

For using the BeamNG ROS integration, a BeamNG.tech build and a python environment with [BeamNGpy][1] installed are required.

Note that BeamNG.tech **only runs on Window**, although Linux support is on its way.
That means that BeamNG.tech needs to run on a separate Windows machine, or that ROS needs to run on on WSL.

The BeamNG ROS integration is compatible with the ROS 1 distributions Melodic Morenia and  Noetic Ninjemys.  

<a name="getstart"></a>

## Getting Started

To use this project, a basic knowledge of the BeamNG.tech simulator and the BeamNGpy is necessary. We recommend to familiarize yourself first with [BeamNGpy][1] to get a basic understanding of the platform before using the BeamNG ROS Integration.

After setting up BeamNG.tech and BeamNGpy with a python environment, the simulation needs to be started through BeamNGpy or through the executable file ```exe```.

**Method 1: Using BeamNGpy**

To start the simulator via Python and BeamNGpy, follow these steps
    - Open your command prompt.
    - Execute the following Python code:


```shell
from beamngpy import BeamNGpy, Scenario, Vehicle
bng = BeamNGpy('localhost', 64256)  # Initialize the BeamNGpy instance to connect to the simulator
bng.open(None, '-gfx', 'dx11', listen_ip='*',launch=True)  # Open the simulator with an open listening IP using DirectX11 renderer, use 'vk' instead of 'dx11' for Vulkan renderer
```

This method initializes the BeamNGpy instance, connecting to the BeamNG.Tech simulator, and then opens the simulator to listen for incoming connections.


**Method 2: Using Command Prompt**

Alternatively, you can start the simulator directly from the command prompt:
    - Open your command prompt.
    - Navigate to your simulator's directory.
    - Paste and execute the following command

```shell
Bin64\BeamNG.tech.x64.exe -gfx dx11 -console -nosteam -tcom-listen-ip 0.0.0.0 -lua extensions.load('tech/techCore');tech_techCore.openServer(64256)
```


The ROS packages from this repository need to be added and build in your catkin workspace.
See the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials) for more information on how to set up a catkin workspace.

A node connecting ROS to the simulation can then be started with the help of the `control.launch` file in the `beamng_control` package through the command:

```shell
roslaunch beamng_control control.launch
```

It needs to be configured to contain the correct IP address of the machine hosting the simulation.
Using it will start up a node that connects to the simulation and starts up a scenario as defined in the `beamng_control/config/simple_scenario.json`.
Other scenario specifications are available in the same directory.

<img src="https://github.com/BeamNG/beamng-ros-integration/raw/master/media/ROS_demo_gif.gif" alt="ROS Demo GIF" width="700" />



## Compatibility  

Running the BeamNG ROS integration requires three individual software components, here is a list of compatible versions.

| BeamNG.tech | BeamNGpy | BeamNG ROS Integration |
|-------------|----------|------------------------|
| 0.33.3      |1.30      | 0.6                    |
| 0.32        |1.29      | 0.5                    |
| 0.31        |1.28      | 0.4.1                  |
| 0.30        |1.27.1    | 0.4                    |
| 0.29        |1.26.1    | 0.3                    |
| 0.28        |1.26      | 0.2.1                  |
| 0.27        |1.25.1    | 0.2                    |
| 0.26        |1.24      | 0.1.1                  |
| 0.25        |1.23.1    | 0.1                    |


## Troubleshooting

This section lists common issues with  BeamNG ROS Integration in particular. Since this
library is closely tied to BeamNG.tech and thus BeamNG.drive, it is also
recommended to consult the documentation on BeamNG.drive here:

[https://documentation.beamng.com/][8]


## Contributions

We always welcome user contributions, be sure to check out our [contribution guidelines][9] first, before starting your work.

[1]: https://github.com/BeamNG/BeamNGpy
[8]: https://documentation.beamng.com/
[9]: https://github.com/BeamNG/BeamNG-ROS-Integration/blob/master/contributing.md