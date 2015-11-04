# Biped
Control of Biped
Link to dynamixel_motor folder
https://github.com/arebgun/dynamixel_motor
First use catkin_make in ~/catkin_ws
With Ros installed open launch folder and edit the controller_manager file to reflect the same usbvalue as for USB2Dynamixel.Save it and using roslaunch execute the file.

Doing this start a new terminal window  and execute start_meta_controller.launch in a similar fashion.
this show show clean exit after initiating all the topics .

Having done this convert all the python file in to executebles using chmod+x *.py
you can run each mode separately in terminal or write a new code imnporting all of them in a new python code.Also if you want you can change the coodinates for the various codes (joint position) it can be done in main section of the code.As of now Cycloidal trajectory has been generated .Cubic spline for master controller yet to be implemented.

Let me know in case you face any difficulty.
