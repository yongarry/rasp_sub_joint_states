#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/String.h>

#include <wiringPi.h>
#include <softPwm.h>
#include <Stepper.h>
#include "bcm2835.h"

// set up your custom GPIO pin
#define Joint1  14

#define Joint3  15
#define Joint4  18
#define Joint5  23
#define GRIPPER 24

//STEPPER MOTOR 
#define STEPS_PER_REVOLUTION 512 //512 steps per revolution
#define IN1     12
#define IN2     16
#define IN3     20
#define IN4     21
#define speed   20              //set your stepper motor speed

//Function: resize joint angle in radian to motor pwm value in int
int resize(float rad)
{
    int value = (rad + 1.57)*(255/3.14);
    
    return value;
}

//Function: resize joint angle in radian to motor pwm value in float
float resizef(float rad)
{
    float value = (rad + 1.57)*(180/3.14);
    
    return value;
}


/* get joint state msg from rostopic /joint_states
    msg.position : postion of the joint
    msg.velocity : velocity of the joint
    msg.name : name of joint
*/
void servoCallback(const sensor_msgs::JointState msg)
{
    
    float rad1 = msg.position[0];
    float rad2 = msg.position[1];
    float rad3 = msg.position[2];
    float rad4 = msg.position[3];
    float rad5 = msg.position[4];
    
    int degree1 = resize(rad1);
    float degree_f1 = resizef(rad1);
    int degree2 = resize(rad2);
    float degree_f2 = resizef(rad2);
    int degree3 = resize(rad3);
    float degree_f3 = resizef(rad3);
    int degree4 = resize(rad4);
    float degree_f4 = resizef(rad4);
    int degree5 = resize(rad5);
    float degree_f5 = resizef(rad5);
    ROS_INFO("degree: [%d],[%f]", degree1,degree_f1);
    
    softPwmWrite(Joint1,degree1);
    Stepper motor2(STEPS_PER_REVOLUTION, IN1, IN2, IN3, IN4);
    motor2.setSpeed(speed);
    motor2.step(degree2);
    softPwmWrite(Joint3,degree3);
    softPwmWrite(Joint4,degree4);
    softPwmWrite(Joint5,degree5);

}


/* get moveit status from rostopic /move_group/status
    msg.status_list[0].status : 1(ACTIVE), 3(SUCCEEDED), 4(ABORTED), 5(REJECTED), 9(LOST)
    msg.status_list[0].text
*/
void statusCallback(const actionlib_msgs::GoalStatusArray msg)
{
    int status_flag = msg.status_list[0].status;
    // string textmsg = msg.status_list[0].text;
    ROS_INFO("[%s]",msg.status_list[0].text.c_str());
    
    //switch (status_flag)
    //{
    //case 3:
    //    softPwmWrite(GRIPPER,128);
    //    break;
    //
    //default:
    //    softPwmWrite(GRIPPER,0);
    //}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "servo_control");
    ros::NodeHandle n;

    wiringPiSetupGpio();

    softPwmCreate(Joint1,0,255);
    
    softPwmCreate(Joint3,0,255);
    softPwmCreate(Joint4,0,255);
    softPwmCreate(Joint5,0,255);
    softPwmCreate(GRIPPER,0,255);

    ros::Subscriber joint_status_sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 10, servoCallback);
    ros::Subscriber move_status_sub = n.subscribe<actionlib_msgs::GoalStatusArray>("/move_group/status", 10, statusCallback);

    ros::spin();
    
    return 0;
}
