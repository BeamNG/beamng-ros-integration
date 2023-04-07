=========
Changelog
=========

Version 0.1.3
===========
- update the code with latest BeamNGpy 1.26 and BeamNG.Tech 0.28
- fix the issue with hand brake and throttle 

Version 0.1.2
===========
- Added ``beamng_teleop_keyboard`` for keyboad control of the vehicle (from ROS) 
- adding ``/cmd_vel`` topic  
- adding hand brake (parking brake) control

Version 0.1.1
===========
- Add zero velocity by pressing space bar 


Version 0.1.0
===========
- Basic bridge with BeamNG.Tech 
- Three ROS packages are released :
    - ``beamng_control`` : To establish the communication and load scenario that can choose the vehicle and the equipped sensors 
    - ``beamng_agent`` : To give ablity to control the vehicle form ROS by simple commands ( move, stop, release -to allow the user control the vehicle from BeamNG.Tech)
    - beamng_msgs : A converter package to adapt beamngpy messages with ROS 