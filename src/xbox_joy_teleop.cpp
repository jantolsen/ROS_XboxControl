#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <xbox_joy/XboxButtons.h>

class XboxJoyTeleop
{
    public:
        XboxJoyTeleop();

    private:

        // Callback function for handling Joystick subscriptions
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

        // ROS node handler
        ros::NodeHandle nh_;

        // ROS Publisher(s)
        // -----------------------
        ros::Publisher xboxJoy_;        // General Xbox Controller

        // ROS Subscriber(s)
        // -----------------------
        ros::Subscriber joySub_;        // Acquire data from Xbox Controller

        // Private variables
        int A_, B_, X_, Y_, LB_, RB_, Back_, Start_, Power_;
        int JoyLeft_PB_, JoyRight_PB_, DPad_UpDn_, DPad_LeftRight_;

};

XboxJoyTeleop::XboxJoyTeleop():

    // Joy-Package mapping
    // (XBOX 360 Wireless - Linux, see http://wiki.ros.org/joy)

      // Buttons
      A_(0),
      B_(1),
      X_(2),
      Y_(3),
      LB_(4),
      RB_(5),
      Back_(6),
      Start_(7),
      Power_(8),
      JoyLeft_PB_(9),
      JoyRight_PB_(10),

      // Axes
      DPad_LeftRight_(6),
      DPad_UpDn_(7)
{
    // Advertise Xbox Buttons to topic
    xboxJoy_ = nh_.advertise<xbox_joy::XboxButtons>("xbox_buttons", 1);

    // Subscribe to Joy topic
    joySub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &XboxJoyTeleop::joyCallback, this);
}

void XboxJoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // Xbox Controller
    // -----------------------
    
    xbox_joy::XboxButtons xboxButton;

    // Mapping from Joy to XboxButtons
    xboxButton.A = joy->buttons[A_];
    xboxButton.B = joy->buttons[B_];
    xboxButton.X = joy->buttons[X_];
    xboxButton.Y = joy->buttons[Y_];
    xboxButton.LB = joy->buttons[LB_];
    xboxButton.RB = joy->buttons[RB_];
    xboxButton.BACK = joy->buttons[Back_];
    xboxButton.START = joy->buttons[Start_];
    xboxButton.POWER = joy->buttons[Power_];
    xboxButton.JoyLeft_PB = joy->buttons[JoyLeft_PB_];
    xboxButton.JoyRight_PB = joy->buttons[JoyRight_PB_];

    xboxButton.DPad_Up = (joy->axes[DPad_UpDn_] > 0.0);
    xboxButton.DPad_Down = (joy->axes[DPad_UpDn_] < 0.0);
    xboxButton.DPad_Left = (joy->axes[DPad_LeftRight_] > 0.0);
    xboxButton.DPad_Right = (joy->axes[DPad_LeftRight_] < 0.0);

    // Publish
    xboxJoy_.publish(xboxButton);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Xbox Joystick Teleop");

    XboxJoyTeleop xbox_joy_teleop;

    ros::spin();

}

