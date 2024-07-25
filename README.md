## SETUP
- mkdir -p dev_ws/src
- cd src
- git clone this repo
- install ros humble desktop
    https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

- install ros humble
```bash
<<<<<<< HEAD
sudo apt install -y ros-humble-desktop
=======
sudo apt install -y ros-humble-desktop 
>>>>>>> main
sudo apt install python3-colcon-common-extensions
```

- add this to .bashrc
```bash
#underlay ros2 setup
source /opt/ros/humble/setup.bash

#colcon cd
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash

#colcon argcomplete
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

#source local bashrc
# if [ !( ./ -ef ~ ) ]; then
if [ "$(realpath $PWD)" != "$(realpath $HOME)" ]; then
# echo $PWD
source $PWD/.bashrc
source install/setup.bash
fi
```

- open a new terminal in dev_ws directory
```bash
sudo apt install python3-rosdep
sudo pip3 install transforms3d
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

## BUILD 
-  build the workspace from dev_ws directory
```bash
colcon build
```

### RUN
- open new terminal from dev_ws or `source dev_ws/install/setup.bash`

1. To setup the sim world
```bash
ros2 launch conveyorbelt_gazebo conveyorbelt.launch.py
```

2. [To start the belt] again open a new terminal as above to source as mentioned above in a new terminal
```bash
ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 10, belt_changer_velocity: 0.5}"
```
3. [To start spawn boxes] again open new terminal as above
```bash
ros2 run ros2_conveyorbelt SpawnObject.py
```