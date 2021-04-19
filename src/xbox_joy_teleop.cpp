#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <xbox_joy/XboxButtons.h>
#include <xbox_joy/XboxAxes.h>

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
        ros::Publisher xboxJoyButtons_;        // General Xbox Controller Buttons
        ros::Publisher xboxJoyAxes_;        // General Xbox Controller Buttons

        // ROS Subscriber(s)
        // -----------------------
        ros::Subscriber joySub_;        // Acquire data from Xbox Controller

        // Private variables
        int A_, B_, X_, Y_, LB_, RB_, Back_, Start_, Power_;
        int JoyLeft_PB_, JoyRight_PB_, DPad_UpDn_, DPad_LeftRight_;
        int JoyLeft_X_, JoyLeft_Y_, JoyRight_X_, JoyRight_Y_, LT_, RT_;
        double JoyScale_;

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
      JoyLeft_X_(0),
      JoyLeft_Y_(1),
      JoyRight_X_(3),
      JoyRight_Y_(4),
      RT_(5),
      LT_(2),
      DPad_LeftRight_(6),
      DPad_UpDn_(7)
{
    // Parameter
    nh_.param("joystick_scale", JoyScale_, 100.0);   // Scaling parameter for joystick values (default = 100.0)

    // Advertise Xbox Buttons to topic
    xboxJoyButtons_ = nh_.advertise<xbox_joy::XboxButtons>("xbox_buttons", 1);
    
    // Advertise Xbox Axes to topic
    xboxJoyAxes_ = nh_.advertise<xbox_joy::XboxAxes>("xbox_axes", 1);

    // Subscribe to Joy topic
    joySub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &XboxJoyTeleop::joyCallback, this);
}

void XboxJoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // Xbox Controller
    // -----------------------
    
    xbox_joy::XboxButtons xboxButton;
    xbox_joy::XboxAxes xboxAxes;

    // Mapping from Joy.Buttons to XboxButtons
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

    // Mapping from Joy.Axes to XboxAxes
    xboxAxes.JoyLeft_X = (-JoyScale_) * joy->axes[JoyLeft_X_];          // Scaling to 0-100% (from -1.0 to 1.0), with Right-Direciton as positive
    xboxAxes.JoyLeft_Y = JoyScale_ * joy->axes[JoyLeft_Y_];
    xboxAxes.JoyRight_X = (-JoyScale_) * joy->axes[JoyRight_X_];        // Scaling to 0-100% (from -1.0 to 1.0), with Right-Direciton as positive
    xboxAxes.JoyRight_Y = JoyScale_ * joy->axes[JoyRight_Y_];
    xboxAxes.RT = ((-1)*(joy->axes[RT_]) + 1.0) * (JoyScale_) / (2);    // Scaling to 0-100% (from 1.0 to -1.0) and changing of sign
    xboxAxes.LT = ((-1)*(joy->axes[LT_]) + 1.0) * (JoyScale_) / (2);    // Scaling to 0-100% (from 1.0 to -1.0) and changing of sign

    // Publish
    xboxJoyButtons_.publish(xboxButton);
    xboxJoyAxes_.publish(xboxAxes);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Xbox Joystick Teleop");

    XboxJoyTeleop xbox_joy_teleop;

    ros::spin();

}

