
markdown
# 🤖 Humanoid_bridge Overview

`Humanoid_bridge` is a ROS 2 package for controlling Unitree and Booster robots (e.g., H1, G1, T1).
It receives commands from the client node, decides whether to perform command interpolation, and, after completing command and state safety checks, sends real-time motor control commands to the robot.

---

## Installation
Prepare the library for T1 robot:
```bash
cd ~
git clone https://github.com/DFKI-SAIROL/booster_robotics_sdk.git
cd booster_robotics_sdk
./install.sh
```
Afer that you can follow the `README` of booster_robotics_sdk to build and prepare the python environment for T1.
Then create your own workspace in your home and enter its `src` folder, for example:
```bash
mkdir -p ~/sairol_ws/src
cd ~/sairol_ws/src
```
Clone this code repository:
```bash
git clone git@github.com:DFKI-SAIROL/humanoid_bridge.git
```
Build the package:
```bash
cd ..
colcon build
```


## Operation guide 
Open a terminal for bridge interface:
```bash
    cd ~/sairol_ws
    source src/humanoid_bridge/setup_booster.sh #if booster T1
    source src/humanoid_bridge/setup_unitree.sh #if unitree G1 or H1
```
then launch the bridge interface:
```bash
    ros2 run robot_bridge T1_bridge --ros-args --params-file src/humanoid_bridge/robot_bridge/params/T1_config.yaml  #if booster T1
    ros2 run robot_bridge G1_bridge --ros-args --params-file src/humanoid_bridge/robot_bridge/params/G1_config.yaml #if unitree G1
    ros2 run robot_bridge H1_bridge --ros-args --params-file src/humanoid_bridge/robot_bridge/params/H1_config.yaml #if unitree H1
```
Now the bridge interface is already launched, and initial mode is damping mode.
Next you can launch your client node. Here we have provided an example for your reference. Open a new terminal for client node: 
```bash 
    cd ~/sairol_ws
    conda activate {YOUR_ENV} # activate your environment relevant to T1 or G1, H1 
    cd src/humanoid_bridge/robot_bridge
    pip install -e .
    cd example/T1
    pip install -r requirements.txt
    cd ~/sairol_ws
    source src/humanoid_bridge/setup_booster.sh #if booster T1
    source src/humanoid_bridge/setup_unitree.sh #if unitree G1 or H1
    
    python src/humanoid_bridge/robot_bridge/example/T1/T1_example.py
```
With this example you can use remote controller to select the mode and use `D-pad (up, down, left, right)` to control the robot movement and press down `either joystick` to stop the movement. Tilt the right joystick to left/right to control the robot yaw rotation. You can see the current velocity in terminal output. 
In case of emergency, use `LT + Back` to switch the mode forcefully to damping mode. clear

Step1: Press `LT + start` to send the `start service` request for starting the control, then bridge can start to publish lowcmd     
        if there is lowcmd from client and checkout custom mode, then move to defauft position for standing.

Step2: Lower the robot’s body and make its feet touch the ground.

Step3: Press `LT + A` to start the policy inference, then robot can use policy to keep standing.

Step4: Use keyboad (w,s,a,d,space) to control the robot

Step5: Press `back` to send the `stop service` request for stopping the control, then bridge can stop to publish any lowcmd and 
        checkout damping mode .


For step 1 you can also use your own joints position for initial joints position with ros2 service command instead of remote controller:
```bash
    ros2 service call /start_control bridge_interface/srv/SetDefaultPosition "{
        default_position: [0, 0,
                            0.2, -1.35, 0, -0.5,
                            0.2, 1.35, 0, 0.5,
                            0,
                            -0.2, 0, 0, 0.4, -0.25, 0,
                            -0.2, 0, 0, 0.4, -0.25, 0]
        }"
```
And for step 5 you can also use ros2 service command instead of remote controller:
```bash
    ros2 service call /stop_control std_srvs/srv/Trigger {}
```

## Other demos
In addition, we have prepared some other demos after you launch `T1_example.py` such as ready position control: 
```bash
    ros2 service call /ready_position_control std_srvs/srv/Trigger {} # you can also press LB of controller
```
And zero position control:
```bash
    ros2 service call /zero_position_control std_srvs/srv/Trigger {}  # you can also press RB of controller  
```
But you should pay attention to stopping policy inference firstly.
If you want to modify the default configuration of the bridge interface like kp/kd limit or tau limit, you can jump into the /robot_bridge/params to modify.

