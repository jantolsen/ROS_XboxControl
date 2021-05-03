#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <xbox_control/XboxButtons.h>
#include <xbox_control/XboxAxes.h>

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
        ros::Publisher xboxJoyButtons_;     // General Xbox Controller Buttons
        ros::Publisher xboxJoyAxes_;        // General Xbox Controller Buttons


        ros::Publisher joint1Command_;
        ros::Publisher joint2Command_;
        ros::Publisher joint3Command_;
        ros::Publisher joint4Command_;
        ros::Publisher joint5Command_;
        ros::Publisher joint6Command_;

        // ROS Subscriber(s)
        // -----------------------
        ros::Subscriber joySub_;            // Acquire data from Xbox Controller

        // Private variables
        int A_, B_, X_, Y_, LB_, RB_, Back_, Start_, Power_;
        int JoyLeft_PB_, JoyRight_PB_, DPad_UpDn_, DPad_LeftRight_;
        int JoyLeft_X_, JoyLeft_Y_, JoyRight_X_, JoyRight_Y_, LT_, RT_;
        double JoyScale_;
        std_msgs::Float64 Joint1_VelocityCommand, Joint2_VelocityCommand, Joint3_VelocityCommand; 
        std_msgs::Float64 Joint4_VelocityCommand, Joint5_VelocityCommand, Joint6_VelocityCommand;

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
    xboxJoyButtons_ = nh_.advertise<xbox_control::XboxButtons>("xbox_buttons", 1);
    
    // Advertise Xbox Axes to topic
    xboxJoyAxes_ = nh_.advertise<xbox_control::XboxAxes>("xbox_axes", 1);

    // Advertise Joint Velocity Commands 
    joint1Command_ = nh_.advertise<std_msgs::Float64>("Joint1_VelocityBased_VelocityController/command", 1);
    joint2Command_ = nh_.advertise<std_msgs::Float64>("Joint2_VelocityBased_VelocityController/command", 1);
    joint3Command_ = nh_.advertise<std_msgs::Float64>("Joint3_VelocityBased_VelocityController/command", 1);
    joint4Command_ = nh_.advertise<std_msgs::Float64>("Joint4_VelocityBased_VelocityController/command", 1);
    joint5Command_ = nh_.advertise<std_msgs::Float64>("Joint5_VelocityBased_VelocityController/command", 1);
    joint6Command_ = nh_.advertise<std_msgs::Float64>("Joint6_VelocityBased_VelocityController/command", 1);

    // Subscribe to Joy topic
    joySub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &XboxJoyTeleop::joyCallback, this);
}

void XboxJoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // Xbox Controller
    // -----------------------
    
    xbox_control::XboxButtons xboxButton;
    xbox_control::XboxAxes xboxAxes;

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

    // Joint Space Teleop:
    // ---------------------------------
        // Joint 1
        if (xboxAxes.RT > 0.0 && xboxAxes.LT == 0)
        {
            Joint1_VelocityCommand.data = xboxAxes.RT * 0.01;
        }
        else if (xboxAxes.LT > 0.0 && xboxAxes.RT == 0)
        {
            Joint1_VelocityCommand.data = (-1) * xboxAxes.LT * 0.01;
        }
        else
        {
            Joint1_VelocityCommand.data = 0.0;
        }

        joint1Command_.publish(Joint1_VelocityCommand);    

        // Joint 2
        Joint2_VelocityCommand.data = xboxAxes.JoyLeft_X * 0.01;
        joint2Command_.publish(Joint2_VelocityCommand); 

        // Joint 3
        Joint3_VelocityCommand.data = xboxAxes.JoyLeft_Y * 0.01;
        joint3Command_.publish(Joint3_VelocityCommand); 

        // Joint 4
        Joint4_VelocityCommand.data = xboxAxes.JoyRight_X * 0.01;
        joint4Command_.publish(Joint4_VelocityCommand); 

        // Joint 5
        Joint5_VelocityCommand.data = xboxAxes.JoyRight_Y * 0.01;
        joint5Command_.publish(Joint5_VelocityCommand); 

         // Joint 6
        if (xboxButton.RB = true && xboxButton.LB == false)
        {
            Joint6_VelocityCommand.data = 0.5;
        }
        else if (xboxButton.RB = false && xboxButton.LB == true)
        {
            Joint6_VelocityCommand.data = -0.5;
        }
        else
        {
            Joint6_VelocityCommand.data = 0.0;
        }
        joint6Command_.publish(Joint6_VelocityCommand); 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Xbox Joystick Teleop");

    XboxJoyTeleop xbox_joy_teleop;

    ros::spin();

}

