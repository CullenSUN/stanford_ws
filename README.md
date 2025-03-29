This code is used for SAC minipupper competition

## terminal 1
```
cd ~
git clone https://github.com/lbaitemple/stanford_ws
cd ~/stanford_ws
colcon build
source ./install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

## terminal 2
```
cd ~/stanford_ws
source ./install/setup.bash 
ros2 launch command_launcher command_launcher.launch.py
```
