# RobotGlobalLocalization

Package for global localization of MiniRys robot.

## Instalation requirements
- Flycapture 2 - camera library
- aruco - marker recognition library
- OpenCV - at least version 3.3

## Running
1. Compile this ros2 package as you normally would using 'colcon build' command.
2. Edit 'blackfly.sh' bash script to use the network interface your camera is connected to.
3. Run 'blackfly.sh' bash script.
4. Run the program you want.
- Programs that doesn't require any arguments
-- camera_test.
- Programs run with one argument - path to 'pointgrey_camera_calibration.yml' file
-- detection_test
-- detection_consistency_test
-- fix_distortion_test
- Programs run with ros2 parameters file - 'global_localization_parameters.yaml'
-- global_localization

Note:
Running programs may require adding opencv and aruco library locations to the LD_LIBRARY_PATH, depending on your instalation location. Example commands:
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<opencv_instalation_location>/lib:<aruco_instalation_location>/lib