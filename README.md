# IK-Leg-Solver
this program will alow you to find the inverse kinematic and forward kinematic for leg with parallel mekanik, and also the trajectory
to run this program you will need 
- ros any version because you'll need the pyKDL
- matplotlib
- numpy

Setup Workspace

Install dependencies
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install ros-noetic-ros-control
sudo apt install ros-noetic-ros-controllers
```
Clone simulation package
```
cd ~
mkdir -p barelangfc/src
cd barelangfc/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Math.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Utility.git
git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
git clone https://github.com/BarelangFC/BarelangFC-AdultSize-Simulation.git
git clone https://github.com/BarelangFC/IK-Leg-Solver.git
cd ..
catkin_make
source devel/setup.bash 
```
Terminal 1 Running Simulation note : after gazebo's open,double click on body frame, and activate Kinematics to prevent robot from falling, click play on beside real time factor
```
cd ~
cd barelangfc
source devel/setup.bash
roslaunch humanoid_gazebo humanoid_gazebo.launch
```
Terminal 2 Running Humanoid Manager
```
cd ~
cd barelangfc
source devel/setup.bash
roslaunch humanoid_manager humanoid_gazebo.launch
```
Terminal 3 Running Rviz
```
cd ~
cd barelangfc
source devel/setup.bash
roslaunch humanoid_description humanoid_display.launch
```
Terminal 4 Running Program
```
cd ~
cd barelangfc/src/Bezier
python3 Trajectory.py 
or
python3 RandomPose.py
```
