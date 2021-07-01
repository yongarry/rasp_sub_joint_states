#include <ros/ros.h>
#include <stdio.h>
#include <wiringPi.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>


#define interval 1000 //(microsecond)

#define IN1 12         //!!!!!!!!!!!!!!!!!!!!!! must change these values
#define IN2 16         //!!!!!!!!!!!!!!!!!!!!!! must change these values
#define IN3 20         //!!!!!!!!!!!!!!!!!!!!!! must change these values
#define IN4 21         //!!!!!!!!!!!!!!!!!!!!!! must change these values


void init_stepper_motor()
{
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(IN3,OUTPUT);
    pinMode(IN4,OUTPUT);
}

void setsteps(int w1, int w2, int w3, int w4)
{
    digitalWrite(IN1,w1);
    digitalWrite(IN2,w2);
    digitalWrite(IN3,w3);
    digitalWrite(IN4,w4);
}

void rotate_forward (int del)
{
    setsteps(1,0,0,0);
    delayMicroseconds(del);
    setsteps(1,1,0,0);
    delayMicroseconds(del);
    setsteps(0,1,0,0);
    delayMicroseconds(del);
    setsteps(0,1,1,0);
    delayMicroseconds(del);
    setsteps(0,0,1,0);
    delayMicroseconds(del);
    setsteps(0,0,1,1);
    delayMicroseconds(del);
    setsteps(0,0,0,1);
    delayMicroseconds(del);
    setsteps(1,0,0,1);
    delayMicroseconds(del);
}

void rotate_backward (int del)
{
    setsteps(1,0,0,0);
    delayMicroseconds(del);
    setsteps(1,0,0,1);
    delayMicroseconds(del);
    setsteps(0,0,0,1);
    delayMicroseconds(del);
    setsteps(0,0,1,1);
    delayMicroseconds(del);
    setsteps(0,0,1,0);
    delayMicroseconds(del);
    setsteps(0,1,1,0);
    delayMicroseconds(del);
    setsteps(0,1,0,0);
    delayMicroseconds(del);
    setsteps(1,1,0,0);
    delayMicroseconds(del);
}

float resizef(float rad)
{
    float value = (rad + 1.57)*(180/3.14);
    
    return value;
}

double total_angle=0;

void subscribe_callback(const sensor_msgs::JointState msg)
{
    float rad2 = msg.position[1];
    float degree2 = resizef(rad2);
    //ROS_INFO("[%f]",total_angle);
    double delta_angle=degree2 - total_angle;
    total_angle=degree2;
    int steps=0;
    // 1 step = 1.8 degrees
    steps = int(delta_angle/1.8+0.5); // rounding process

    if (steps >0)
    {
        for(int i=0;i<steps;i++)
        {
            rotate_forward(interval);       //check whether it rotates opposite direction. 
                                            //if so, change this rotate_forward to rotate_backward    
                                            //and change below rotate_backward to rotate_forward
            ROS_INFO("[%d]",steps);
        }
    }
    else
    {
        steps*=-1;
        for(int i=0;i<steps;i++)
        {
            rotate_backward(interval);
            ROS_INFO("[%d]",steps);
        }
    }
}


int main(int argc, char** argv)
{
    wiringPiSetupGpio();
    init_stepper_motor();
    ros::init(argc, argv,"joint2_control");
    ros::NodeHandle nh;
        
    ros::Subscriber angle_reciever=nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, subscribe_callback);
    
    ros::spin();

    return 0;
}
