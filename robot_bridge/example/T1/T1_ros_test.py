#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import numpy as np
import time
import yaml
import logging
import threading
import signal
import sys
import atexit
from bridge_interface.msg import RobotCmd, MotorCmd
# from bridge_interface.srv import SetDefaultPosition
from booster_interface.msg import LowState #,LowCmd

from std_srvs.srv import Trigger

from utils.command import create_prepare_cmd, create_first_frame_rl_cmd, init_Cmd_T1, create_stop_frame_rl_cmd
from utils.remote_control_service import RemoteControlService
from utils.rotate import rotate_vector_inverse_rpy
from utils.timer import TimerConfig, Timer
from utils.policy import Policy

# 导入RobotClient
import os
script_path = '/home/xuanhaosong/sairol_ws/src/sairol_bridge/robot_bridge/scripts'
if script_path not in sys.path:
    sys.path.append(script_path)
from robot_client import RobotClient



from booster_robotics_sdk_python import (
    ChannelFactory,
    B1LocoClient,
    B1LowCmdPublisher,
    B1LowStateSubscriber,
    # LowCmd,
    # LowState,
    # B1JointCnt,
    RobotMode,
    LowCmdType
)


B1JointCnt = 23 

class Controller(Node):
    def __init__(self, cfg_file) -> None:
        super().__init__('t1_controller_node')
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        # Load config
        with open(cfg_file, "r", encoding="utf-8") as f:
            self.cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

        # Initialize components
        self.remoteControlService = RemoteControlService()
        self.policy = Policy(cfg=self.cfg)

        self._init_timer()
        
        # 使用RobotClient替代直接的ROS通信
        self.robot_client = RobotClient("T1", B1JointCnt, self)

        self.running = True
        
        # 初始化一些需要的变量
        self.dof_target = np.zeros(B1JointCnt, dtype=np.float32)

        self.low_cmd = RobotCmd()
    def _init_timer(self):
        self.timer = Timer(TimerConfig(time_step=self.cfg["common"]["dt"]))
        self.next_publish_time = self.timer.get_time()
        self.next_inference_time = self.timer.get_time()


    def _send_cmd(self, dof_target_pos, dof_target_vel, dof_target_tau, dof_target_kp, dof_target_kd):
        
        self.robot_client._send_cmd(dof_target_pos, dof_target_vel, dof_target_tau, dof_target_kp, dof_target_kd)

    def cleanup(self) -> None:
        self.logger.info("Cleaning up resources...")
        self.running = False
        self.remoteControlService.close()
        

        if hasattr(self, 'cmd_timer'):
            self.cmd_timer.cancel()
            

        if hasattr(self, "publish_runner") and getattr(self, "publish_runner") is not None:
            self.publish_runner.join(timeout=1.0)

    def start_rl_gait_conditionally(self):
        print(f"{self.remoteControlService.get_rl_gait_operation_hint()}")
        while True:
            if self.remoteControlService.start_rl_gait():
                break
            time.sleep(0.1)
            
        create_first_frame_rl_cmd(self.low_cmd, self.cfg)
        
  
        dof_target_pos = np.array([self.low_cmd.motor_cmd[i].q for i in range(B1JointCnt)])
        dof_target_vel = np.array([self.low_cmd.motor_cmd[i].dq for i in range(B1JointCnt)])
        dof_target_tau = np.array([self.low_cmd.motor_cmd[i].tau for i in range(B1JointCnt)])
        dof_target_kp = np.array([self.low_cmd.motor_cmd[i].kp for i in range(B1JointCnt)])
        dof_target_kd = np.array([self.low_cmd.motor_cmd[i].kd for i in range(B1JointCnt)])
        
        self._send_cmd(dof_target_pos, dof_target_vel, dof_target_tau, dof_target_kp, dof_target_kd)

        self.next_inference_time = self.timer.get_time()
        self.next_publish_time = self.timer.get_time()
        
        print(f"{self.remoteControlService.get_operation_hint()}")

    def run(self):
        self.timer.counter = self.robot_client.count
        time_now = self.timer.get_time()

        if time_now < self.next_inference_time:
            time.sleep(0.001)
            return
   
        if abs(self.robot_client.rpy[0]) > 1.0 or abs(self.robot_client.rpy[1]) > 1.0:
            self.logger.warning("IMU base rpy values are too large: {}".format(self.robot_client.rpy))
            self.running = False
            return
        
        self.logger.debug("-----------------------------------------------------")
        self.next_inference_time += self.policy.get_policy_interval()
        self.logger.debug(f"Next start time: {self.next_inference_time}")
        start_time = time.perf_counter()

        self.dof_target[:] = self.policy.inference(
            time_now=time_now,
            dof_pos=self.robot_client.dof_pos,
            dof_vel=self.robot_client.dof_vel,
            base_ang_vel=self.robot_client.base_ang_vel,
            projected_gravity=self.robot_client.projected_gravity,
            vx=self.remoteControlService.get_vx_cmd(),
            vy=self.remoteControlService.get_vy_cmd(),
            vyaw=self.remoteControlService.get_vyaw_cmd(),
        )

        inference_time = time.perf_counter()
        self.logger.debug(f"Inference took {(inference_time - start_time)*1000:.4f} ms")

      
        dof_target_pos = self.dof_target.copy()
        dof_target_vel = np.zeros(B1JointCnt, dtype=np.float32)
        dof_target_tau = np.zeros(B1JointCnt, dtype=np.float32)
        dof_target_kp = np.array([float(self.cfg["common"]["stiffness"][i]) for i in range(B1JointCnt)], dtype=np.float32)
        dof_target_kd = np.array([float(self.cfg["common"]["damping"][i]) for i in range(B1JointCnt)], dtype=np.float32)
        
 
        self._send_cmd(dof_target_pos, dof_target_vel, dof_target_tau, dof_target_kp, dof_target_kd)
        time.sleep(0.001)

    def __enter__(self) -> "Controller":
        return self

    def __exit__(self, *args) -> None:
        self.cleanup()




if __name__ == "__main__":
    import argparse
    import signal
    import sys
    import os

    def signal_handler(sig, frame):
        print("\nShutting down...")
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    cfg_file = "src/sairol_bridge/robot_bridge/example/T1/configs/T1.yaml"
    print(f"Starting ROS 2 custom controller...")
    
    rclpy.init()

    try:
        controller = Controller(cfg_file)

        time.sleep(2)  
        print("Initialization complete.")

        controller.start_rl_gait_conditionally()
        i = 0
        try:
            while controller.running and rclpy.ok():
                controller.run()

        except KeyboardInterrupt:
            print("\nKeyboard interrupt received. Cleaning up...")
        finally:
            controller.cleanup()
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()