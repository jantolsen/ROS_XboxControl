# ros_XboxController

Dependencies:
Following ros-package is used as XboxControl-Driver
http://wiki.ros.org/joy

Button and Axes mapping may need alteration according to type of Xbox-Controller used

Joystick configuration:
http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick
Check which joystick-number is assigned by Linux (js0 is used in this repo)

$ ls /dev/input/
  js0, js1, js2, jsX, etc
  
Test joystick  
$ sudo jstest /dev/input/jsX


Alter the "xbox_control.launch.launch" file for your project
<param name="dev" type="string" value="/dev/input/jsX" />


