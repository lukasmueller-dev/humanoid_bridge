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
        self._init_low_state_values()
        self._init_ros_communication()

        self.running = True


    def _init_timer(self):
        self.timer = Timer(TimerConfig(time_step=self.cfg["common"]["dt"]))
        self.next_publish_time = self.timer.get_time()
        self.next_inference_time = self.timer.get_time()

    def _init_low_state_values(self):
        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.projected_gravity = np.zeros(3, dtype=np.float32)
        self.dof_pos = np.zeros(B1JointCnt, dtype=np.float32)
        self.dof_vel = np.zeros(B1JointCnt, dtype=np.float32)

        self.dof_target = np.zeros(B1JointCnt, dtype=np.float32)
        self.filtered_dof_target = np.zeros(B1JointCnt, dtype=np.float32)
        self.dof_pos_latest = np.zeros(B1JointCnt, dtype=np.float32)

    def _init_ros_communication(self) -> None:

        try:

            self.low_state_subscription = self.create_subscription(
                LowState,
                '/low_state',
                self._low_state_handler,
                1
            )
            
            self.low_cmd_publisher = self.create_publisher(
                RobotCmd,
                '/robot_cmd',
                1
            )
            
            # self.client = B1LocoClient()
            # self.client.Init()

            self.logger.info("ROS 2 communication initialized successfully")
        except Exception as e:
            self.logger.error(f"Failed to initialize ROS 2 communication: {e}")
            raise

    def _low_state_handler(self, low_state_msg: LowState):

        if abs(low_state_msg.imu_state.rpy[0]) > 1.0 or abs(low_state_msg.imu_state.rpy[1]) > 1.0:
            self.logger.warning("IMU base rpy values are too large: {}".format(low_state_msg.imu_state.rpy))
            self.running = False
            
        self.timer.tick_timer_if_sim()
        time_now = self.timer.get_time()
        
     
        for i, motor in enumerate(low_state_msg.motor_state_serial):
            if i < B1JointCnt:
                self.dof_pos_latest[i] = motor.q
                
        if time_now >= self.next_inference_time:
            self.projected_gravity[:] = rotate_vector_inverse_rpy(
                low_state_msg.imu_state.rpy[0],
                low_state_msg.imu_state.rpy[1],
                low_state_msg.imu_state.rpy[2],
                np.array([0.0, 0.0, -1.0]),
            )
            self.base_ang_vel[:] = low_state_msg.imu_state.gyro
            for i, motor in enumerate(low_state_msg.motor_state_serial):
                if i < B1JointCnt:
                    self.dof_pos[i] = motor.q
                    self.dof_vel[i] = motor.dq

    def _send_cmd(self, cmd: RobotCmd):
        self.low_cmd_publisher.publish(cmd)

    def cleanup(self) -> None:
        self.logger.info("Cleaning up resources...")
        self.running = False
        self.remoteControlService.close()
        

        if hasattr(self, 'cmd_timer'):
            self.cmd_timer.cancel()
            

        if hasattr(self, "publish_runner") and getattr(self, "publish_runner") is not None:
            self.publish_runner.join(timeout=1.0)

    def start_custom_mode_conditionally(self):

        print(f"{self.remoteControlService.get_custom_mode_operation_hint()}")
        while True:
            if self.remoteControlService.start_custom_mode():
                break
            time.sleep(0.1)
            
        start_time = time.perf_counter()
        

        self.low_cmd = RobotCmd()
        create_prepare_cmd(self.low_cmd, self.cfg)
        
        for i in range(B1JointCnt):
            self.dof_target[i] = self.low_cmd.motor_cmd[i].q
            self.filtered_dof_target[i] = self.low_cmd.motor_cmd[i].q
            

    def start_rl_gait_conditionally(self):

        print(f"{self.remoteControlService.get_rl_gait_operation_hint()}")
        while True:
            if self.remoteControlService.start_rl_gait():
                break
            time.sleep(0.1)
            
        create_first_frame_rl_cmd(self.low_cmd, self.cfg)
        self._send_cmd(self.low_cmd)
        
        self.next_inference_time = self.timer.get_time()
        self.next_publish_time = self.timer.get_time()
        
        print(f"{self.remoteControlService.get_operation_hint()}")

    def run(self):
        time_now = self.timer.get_time()
        if time_now < self.next_inference_time:
            time.sleep(0.001)
            return
        self.logger.debug("-----------------------------------------------------")
        self.next_inference_time += self.policy.get_policy_interval()
        self.logger.debug(f"Next start time: {self.next_inference_time}")
        start_time = time.perf_counter()

        self.dof_target[:] = self.policy.inference(
            time_now=time_now,
            dof_pos=self.dof_pos,
            dof_vel=self.dof_vel,
            base_ang_vel=self.base_ang_vel,
            projected_gravity=self.projected_gravity,
            vx=self.remoteControlService.get_vx_cmd(),
            vy=self.remoteControlService.get_vy_cmd(),
            vyaw=self.remoteControlService.get_vyaw_cmd(),
        )

        inference_time = time.perf_counter()
        self.logger.debug(f"Inference took {(inference_time - start_time)*1000:.4f} ms")

        for i in range(B1JointCnt):
            self.low_cmd.motor_cmd[i].q = float(self.dof_target[i])
            self.low_cmd.motor_cmd[i].tau = 0.0  # Reset torque to zero
            self.low_cmd.motor_cmd[i].kp = float(self.cfg["common"]["stiffness"][i])
            self.low_cmd.motor_cmd[i].kd = float(self.cfg["common"]["damping"][i])
        
        self._send_cmd(self.low_cmd)
        time.sleep(0.001)

    def __enter__(self) -> "Controller":
        return self

    def __exit__(self, *args) -> None:
        self.cleanup()


def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except ExternalShutdownException:
        pass

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
    # ChannelFactory.Instance().Init(0)
    
    rclpy.init()

    t = threading.Thread(target=spin_in_background)
    t.start()
    

    try:
        controller = Controller(cfg_file)
        rclpy.get_global_executor().add_node(controller)

        time.sleep(2)  
        print("Initialization complete.")
        
        controller.start_custom_mode_conditionally()
        controller.start_rl_gait_conditionally()

        try:
            while controller.running and rclpy.ok():
                controller.run()
                # rclpy.spin_once(controller, timeout_sec=0.001)
            
          
            # controller.client.ChangeMode(RobotMode.kDamping)
            
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received. Cleaning up...")
        finally:
            controller.cleanup()
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()