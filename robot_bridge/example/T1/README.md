# Deploy on Booster Robot

## Installation

Follow these steps to set up your environment:

1. Install Python dependencies:

    ```sh
    $ pip install -r requirements.txt
    ```

2. Install Booster Robotic SDK:

    Refer to the [Booster Robotics SDK Guide](https://booster.feishu.cn/wiki/DtFgwVXYxiBT8BksUPjcOwG4n4f#share-WDzedC8AiovU8gxSjeGcQ5CInSf) and ensure you complete the section on [Compile Sample Programs and Install Python SDK](https://booster.feishu.cn/wiki/DtFgwVXYxiBT8BksUPjcOwG4n4f#share-EI5fdtSucoJWO4xd49QcE5JxnCf).

## Usage

1. Prepare the robot:

    - Power on the robot and switch on the bridge_main_node with:
    
    ```sh
    ros2 run robot_bridge bridge_main --ros-args --params-file /your_path_to_sairol_ws/sairol_ws/src/sairol_bridge/robot_bridge/params/T1_config.yaml
    ```
    Then place the robot to a stable standing position on the ground, switch to start control mode with remote controller or using service command.

2. Run the deployment script:

    ```sh
    $ python T1_policy_deploy.py --config=T1.yaml 
    ```

    - `--config`: Name of the configuration file, located in the `configs/` folder.
    - `--net`: Network interface for SDK communication. Default is `127.0.0.1`.

3. Exit Safely:

    Switch back to PREP Mode before terminating the program to safely release control.
