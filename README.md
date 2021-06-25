# rasp_sub_joint_states

Subscribes joint_states from moveit and give servo control by raspberry

## Dependecies

Need ros-melodic (click [here](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi)) installed in rasbian

  1. Do step `2.Prerequisites` 
  
  for recieving key failure try this. `wget http://packages.ros.org/ros.key -O - | sudo apt-key add -`

  2. For `3.Installation` follow this.
  
    $ mkdir -p ~/ros_catkin_ws
    $ cd ~/ros_catkin_ws
    $ rosinstall_generator ros_comm common_msgs --rosdistro melodic --deps --wet-only --tar > melodic-ros_comm-wet.rosinstall
    $ wstool init src melodic-ros_comm-wet.rosinstall
    $ cd ~/ros_catkin_ws
    $ rosdep install -y --from-paths src --ignore-src --rosdistro melodic -r --os=debian:buster
    $ sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j2
    $ source /opt/ros/melodic/setup.bash
    $ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
  
  3. Add your own catkin workspace
  
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws && catkin_make
    $ source ~/catkin_ws/devel/setup.bash
    $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    
  4. Ros connect Two machines(master PC & raspberry)
    For instruction click [here](https://blog.iolate.kr/225#:~:text=ROS%20%EB%8A%94%20TCP%20layer%EC%97%90%EC%84%9C,%ED%95%9C%EB%8B%A4%EC%9D%8C%EC%97%90%20%EC%84%A4%EC%A0%95%EC%9D%84%20%ED%95%B4%EC%A3%BC%EB%A9%B4%20%EB%90%A8.).
   
  5. [wiringPi](https://roboticsbackend.com/introduction-to-wiringpi-for-raspberry-pi/)
    Need to download wiring pi package
    
    $ sudo apt-get install wiringpi

## How to build

    $ cd ~/catkin_ws
    $ catkin_make
    
## Run

  1. Make sure you run `roslaunch armrobot_moveit demo.launch` on your master PC. (Also make sure your PC and raspberry is connect with ROS.) 

    $ rosrun rasp_sub_joint_states servo_control
    
