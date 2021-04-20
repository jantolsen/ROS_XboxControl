# ROS_XboxControl

Dependencies:
Following ros-package is used as XboxControl-Driver
http://wiki.ros.org/joy

Button and Axes mapping may need alteration according to type of Xbox-Controller used

Joystick configuration:
http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick
Check which joystick-number is assigned by Linux (js0 is used in this repo):
  $ ls /dev/input/
    results:
        js0, js1, js2, jsX, etc
  
Test joystick  
  $ sudo jstest /dev/input/jsX

Joystick permission and accessibility:
http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

List permissions of the joystick:
  $ ls -l /dev/input/jsX
    Results: 
        crw-rw-XX- 1 root dialout 188, 0 2021-04-19 20:15 /dev/input/jsX
  
If XX is rw: the js device is configured properly
If XX is --: the js device is not configured properly and you need to:
  $ sudo chmod a+rw /dev/input/jsX

Alter the "xbox_control.launch.launch" file for your project
<param name="dev" type="string" value="/dev/input/jsX" />


