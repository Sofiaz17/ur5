<p align='center'>
    <h1 align="center">FUNDAMENTALS OF ROBOTICS PROJECT </h1>
    <p align="center">
    University of Trento A.Y. 2023/2024
    </p>
    <p align='center'>
    Developed by:<br>
    Alessandro De Vidi <br>
    Daniele Marisa <br>
    Giulia Modenese <br>
    Sofia Zandonà
    </p>   
</p>

## PROJECT DESCRIPTION
The objective of the project is to use the manipulator able to do autonomus pick-and-place operations.
Through the zed-camera, able to detect the classes and position of every block, the robot have to pick the object and place in the corrisponding potion.
## FOLDER STRUCTURE

## INSTALLATION
The project has been developed and tested on Ubuntu 20.04 with ROS Noetic, also we used the [locosim](https://github.com/mfocchi/locosim) repository for the ur5 simulation. The installation of the project is the following:
1) Clone the [locosim repository](https://github.com/mfocchi/locosim) and follow the respective instructions to install it
2) Clone this [repository](https://github.com/Sofiaz17/ur5.git) in the `catkin_ws/src` folder of the catkin workspace
3) Install the vision dependencies with the following command:
- Install YOLOv5 dependencies
   
```BASH
cd ~
git clone https://github.com/ultralytics/yolov5.git
cd yolov5
pip3 install -r requirements.txt
```
- Intall the other dependencies
```BASH
pip install torchvision==0.13.0
```
4) Compile the project with the following command:
```BASH
cd ~/catkin_ws
catkin_make
source install/setup.bash
```

## HOW TO RUN THE PROJECT
### SETUP
Inside  ``~/ros_ws/src/locosim/robot_control/base_controllers/params.py`` go to the line 46 and set:
```
'gripper_sim': True,
```

Then we copy the models inside the worlds directory :
```
cd ~/catkin_ws/src/ur5
cp -r Models ~/ros_ws/src/locosim/ros_impedance_controller/worlds
```
Now add the world.world file
```BASH
cp world.world ~/ros_ws/src/locosim/ros_impedance_controller/worlds
```
Last thing is to modify the ur5_generic.py file in the locosim project adding the following line at line 71
```PYTHON
self.world_name = 'world.world'
```
Now we are able to run the project.

### RUN
For running the project you need to run the following commands:
1) Run in one window the ur5 script with the following command:
```BASH
rosrun ur5 ur5_generic.py
```
2) Run in another window the ROS nodes with the following command:
```BASH
roslaunch ur5 ur5.launch
```
