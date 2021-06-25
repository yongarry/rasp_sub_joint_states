#include "ros/ros.h"
#include <sensor_msgs/JointState.h>

// #include <wiringPi.h>
// #include <softPwm.h>

#define Joint1  3
#define Joint2  4
#define Joint3  5
#define Joint4  6
#define Joint5  7

float resize(float rad)
{
    float value = (rad + 1.57)*(180/3.14);
    return value;
}

void servoCallback(const sensor_msgs::JointState msg)
{
    ROS_INFO("I heard: [%f]", msg.position[0]);
    float rad1 = msg.position[0];
    float degree = resize(rad1);
    ROS_INFO("degree: [%f]", degree);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "servo_control");
    ros::NodeHandle n;

    // wiringPiSetupGpio();

    // softPwmCreate(Joint1,0,255);
    // softPwmCreate(Joint2,0,255);
    // softPwmCreate(Joint3,0,255);
    // softPwmCreate(Joint4,0,255);
    // softPwmCreate(Joint5,0,255);


    ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 10, servoCallback);

    ros::spin();
    
    return 0;
}