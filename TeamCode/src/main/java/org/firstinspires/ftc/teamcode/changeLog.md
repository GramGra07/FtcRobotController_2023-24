5.5.0 January 26th, 2024
Added PIDF modes and changed auto to a Finite State Machine

5.1.0 January 18th, 2024
Added LoopTime class to calculate loops per second and the refresh rate of the robot systems,
specifically distance sensors

5.0.0 December 27th, 2023
Added three new Autonomous Modes for competition use
Added all of the preselect teleOp modes

4.3.0 November 30th, 2023
Added new autonomous in teleOp functionality with async trajectories

4.2.0 November 4th, 2023
Updated to FTC 9.0.1

4.1.0 October 30th, 2023
Added disableRR.sh and enableRR.sh and documentation for these. They enable all Road Runner opModes
or disable all of them. rrFiles.txt controls which files are changed. This is a shell script and
should run natively on all devices without changing anything.

4.0.1 October 28th, 2023
Updated to fit all motors, config, and keybinds for the 2023-2024 season
Created new Operator class for second gamepad

4.0.0 September 9th, 2023
Updated to FTC 9.0 and start of season

3.0.1 July 16th,2023
Added a Math Functions class that contains easy to use functions including math

3.0.0 July 11th,2023
FTC has officially updated to 8.2.0 code is all updated to work with this version

2.5.1 July 10th,2023
Added a new class called varConfig that contains all changeable variables for the robot
These can be changed easily using the FTC Dashboard and won't be saved until enter is pressed, then
it will change the variables live, then the variables need to be changed in the actual code
Started on Object Recognition using OpenCV with ConeObjVars class containing all variables for the
cone object, can also be changed with FTC Dashboard

2.5.0 July 1st,2023
Added data logging for paths
Commands are in HardwareConfig (write to file), once connected to robot,
run the two commands and paste the data into file.txt, then run the python script
Python script will take the file data and make a graph and show it
Instructions are also all in the HardwareConfig file under writeToFile()

2.1.0 June 28th,2023
Road Runner is now fully implemented and tested
Road Runner Localization implemented into teleOp
Current pose is now saved through every opMode
Start pose will only ever be set in AutoHardware
Blank opModes are ready and available for use (copy and paste them, rename, add code)

2.0.0 June 26th,2023
Added Road Runner implementations and are tested

1.2.2 June 22nd,2023
More OpenCV pipelines available for use along with testing images for some of the pipelines
New pipelines.txt file illustrating all of the pipelines available

1.2.0 June 13th,2023
Added a new OpenCV Pipeline class that contains all the used pipelines
Added the pipelineTester.java program to test the pipelines
Added edge detection, color detection, and these combined as well as one with bounding boxes to
create
Color detection has four detectable colors: red, blue, yellow, and green

1.1.0 June 12th,2023
Added config.md and a template
OpenCV tests working (and april tags) and will switch with vuforia as well  
blink class completely tested and working

1.0.1 June 5th,2023
Finished Potentiometer and Limit Switches
New blink class with everything to control the Blinkin driver

1.0.0 June 5th,2023
Initial Versioning