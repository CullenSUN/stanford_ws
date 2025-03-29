This code is used for SAC minipupper competition


## terminal 1
```
cd ~
git clone https://github.com/lbaitemple/stanford_ws
cd ~/stanford_ws
python JupyterServer.py
```

## terminal 2
```
cd ~/stanford_ws
colcon build
source ./install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

## terminal 3
```
cd ~/stanford_ws
source ./install/setup.bash 
ros2 launch command_launcher command_launcher.launch.py
```

## terminal 4 (test windows)
you can send a pub message for different action, which the camera message needs to generate 
the message
```
cd ~/stanford_ws
source ./install/setup.bash 
ros2 topic pub --once /command_topic std_msgs/msg/String data:\ \'dance1\'\ 
```

or 

```
cd ~/stanford_ws
source ./install/setup.bash 
ros2 topic pub --once /command_topic std_msgs/msg/String data:\ \'dance2\'\ 
```

