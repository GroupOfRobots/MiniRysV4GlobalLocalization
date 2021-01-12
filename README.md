# RobotGlobalLocalization

Package for global localization of MiniRys robot.

## Instalation requirements
- Flycapture 2 - camera library
- aruco - marker recognition library
- OpenCV - at least version 3.3

## Running
To run the included programs compile ros2 package as you normally would.
Note:
Running programs may require adding opencv and aruco library locations to the LD_LIBRARY_PATH, depending on your instalation location. Example commands:
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<opencv_instalation_location>/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<aruco_instalation_location>/lib