# Sawyer XR Teleop

This project is a 4th year Mechatronics FYDP completed by Jeffrey Lee, Tiffany Ng, Kaitlyn Lee, and Josh Blatt. This project uses the Sawyer robot arm and the Leap Motion Controller for teleoperation. 

## Contents

This project is composed of 3 main components: the Sawyer Arm Controller, the Leap Data Processor, and the iControBot UI. 
- SawyerArmController.py is located in src/sawyer_xr_teleop. 
- LeapDataProcessor.py is located in src/sawyer_xr_teleop/LeapSDK/samples
- iControBot_UI.py is located in src/sawyer_xr_teleop

There are other supporting ros message files, and UI files. The rest are the sample code used to create our files.

## Running the project on AirLab2
Open a separate terminal for each of the following steps. Ensure you run the ./intera.sh file in fydp_ws to source and you are on the AIRLAB wifi or change the ./intera.sh file to match the Sawyer network you are using.

1. `sudoLeapd` to start the leap Controller
2. `LeapControlPanel --showsettings` will open a control panel where you can open the diagnostic visualizer to see the Leap camera's wireframe
3. `python SawyerArmController.py` in the aforementioned location to run the Sawyer Controller
4. `python LeapDataProcessor.py` in the aforementioned location to run the Leap data processor
5. `python iControBot_UI.py` in the adorementioned location to run the UI

Steps 1,3, and 4 are the bare minimum files you need to run to start the controller. 

## Using the controller

To start the controller, place your hand within the view of the camera with your palm facing the camera and make a circular twirl motion your index finger in to start the controller. The end effector of the robot arm should now start following your palm position. To stop the controller, create a fist. 

## System Architecture

You can view this in the System Architecture slide in the Jeffrey FYDP - AIR Lab Presentation.pptx file.





