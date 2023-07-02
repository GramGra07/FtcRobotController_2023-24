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
New pipelins.txt file illustrating all of the pipelines available

1.2.0 June 13th,2023
Added a new OpenCV Pipeline class that contains all the used pipelines
Added the pipelineTester.java program to test the pipelines
Added edge detection, color detection, and these combined as well as one with bounding boxes to create
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