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
from bridge_interface.srv import SetDefaultPosition
from booster_interface.msg import LowState
from std_srvs.srv import Trigger

from utils.command import create_prepare_cmd, create_first_frame_rl_cmd, init_Cmd_T1
from utils.remote_control_service import RemoteControlService
from utils.rotate import rotate_vector_inverse_rpy
from utils.timer import TimerConfig, Timer
from utils.policy import Policy


B1JointCnt = 23  # Number of joints in the B1 robot



class T1ClientNode:
    def __init__(self, nh, config):
        self.nh = nh
        self.config = config
        self.rate = self.nh.create_rate(1 / (config["common"]["dt"] * config["policy"]["control"]["decimation"]))
        self.rate._timer.reset()

        # Initialize components
        self.remoteControlService = RemoteControlService()
        self.policy = Policy(cfg=self.config)

        self.logger = self.nh.get_logger()

        self.running = True

        self._init_timer()
        self._init_low_state_values()
        self._init_communication()

        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _init_low_state_values(self):
        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.projected_gravity = np.zeros(3, dtype=np.float32)
        self.dof_pos = np.zeros(B1JointCnt, dtype=np.float32)
        self.dof_vel = np.zeros(B1JointCnt, dtype=np.float32)

        self.dof_target = np.zeros(B1JointCnt, dtype=np.float32)
        self.filtered_dof_target = np.zeros(B1JointCnt, dtype=np.float32)
        self.dof_pos_latest = np.zeros(B1JointCnt, dtype=np.float32)
        
        self.robot_cmd = RobotCmd()
        init_Cmd_T1(self.robot_cmd)
    
    def _init_timer(self):
        self.timer = Timer(TimerConfig(time_step=self.config["common"]["dt"]))
        self.next_publish_time = self.timer.get_time()
        self.next_inference_time = self.timer.get_time()
    
    def _init_communication(self) -> None:
        try:
            # Create ROS 2 subscriber and publisher
            self.low_state_subscriber = self.nh.create_subscription(LowState, '/low_state', self._low_state_handler, 1)
            self.robot_cmd_publisher = self.nh.create_publisher(RobotCmd, '/robot_cmd', 1)
            
        except Exception as e:
            self.logger.error(f"Failed to initialize communication: {e}")
            raise

    def _low_state_handler(self, low_state_msg: LowState):
        if abs(low_state_msg.imu_state.rpy[0]) > 1.0 or abs(low_state_msg.imu_state.rpy[1]) > 1.0:
            self.logger.warning("IMU base rpy values are too large: {}".format(low_state_msg.imu_state.rpy))
            self.running = False
            return
        self.timer.tick_timer_if_sim()
        for i, motor in enumerate(low_state_msg.motor_state_serial):
            # print(f"Motor {i} state: q={motor.q}, dq={motor.dq}")
            self.dof_pos_latest[i] = motor.q
            self.dof_pos[i] = motor.q
            self.dof_vel[i] = motor.dq

        self.projected_gravity[:] = rotate_vector_inverse_rpy(
            low_state_msg.imu_state.rpy[0],
            low_state_msg.imu_state.rpy[1],
            low_state_msg.imu_state.rpy[2],
            np.array([0.0, 0.0, -1.0]),
        )
        self.base_ang_vel[:] = low_state_msg.imu_state.gyro

    def publish_cmd(self):
        time_now = self.nh.get_clock().now()
        time_now = time_now.nanoseconds / 1e9  # Convert to seconds
        # time_now = self.timer.get_time()
        print("Current time", time_now)
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
        
        self.filtered_dof_target = self.filtered_dof_target * 0.8 + self.dof_target * 0.2

        for i in range(B1JointCnt):
            self.robot_cmd.motor_cmd[i].q = float(self.filtered_dof_target[i])

        # Use series-parallel conversion for torque to avoid non-linearity
        for i in self.config["mech"]["parallel_mech_indexes"]:
            self.robot_cmd.motor_cmd[i].q = float(self.dof_pos_latest[i])
            self.robot_cmd.motor_cmd[i].tau = float(np.clip(
                (self.filtered_dof_target[i] - self.dof_pos_latest[i]) * self.config["common"]["stiffness"][i],
                -self.config["common"]["torque_limit"][i],
                self.config["common"]["torque_limit"][i],
            ))
            self.robot_cmd.motor_cmd[i].kp = 0.0
        self.robot_cmd_publisher.publish(self.robot_cmd)


    def start(self):
        print(f"{self.remoteControlService.get_rl_gait_operation_hint()}")
        while True:
            if self.remoteControlService.start_rl_gait():
                break
            time.sleep(0.1)
       
        
        # Call start control service
        print("Starting control...")
        try:
            client = self.nh.create_client(SetDefaultPosition, '/start_control')
            if client.wait_for_service(timeout_sec=1.0):
                request = SetDefaultPosition.Request()
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self.nh, future, timeout_sec=1.0)
                if future.result() is not None:
                    self.logger.info(f"Start service called successfully: {future.result().message}")
                else:
                    self.logger.warning("Start service call failed")
            else:
                self.logger.warning("Start service not available")
        except Exception as e:
            self.logger.error(f"Failed to call start service: {e}")
        
        # create_first_frame_rl_cmd(self.robot_cmd, self.config)
        # self.robot_cmd_publisher.publish(self.robot_cmd)


        print(f"{self.remoteControlService.get_operation_hint()}")
        time.sleep(5.0)  # Wait for the first frame command to be published

        while self.running:
            self.publish_cmd()
            # self.logger.info("Publishing command...")
            self.rate.sleep()
            # time.sleep(0.001)


        # Call stop control service to stop
        print("Stopping control...")
        try:
            client = self.nh.create_client(Trigger, '/stop_control')
            if client.wait_for_service(timeout_sec=1.0):
                request = Trigger.Request()
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self.nh, future, timeout_sec=1.0)
                if future.result() is not None:
                    self.logger.info(f"Stop service called successfully: {future.result().message}")
                else:
                    self.logger.warning("Stop service call failed")
            else:
                self.logger.warning("Stop service not available")
        except Exception as e:
            self.logger.error(f"Failed to call stop service: {e}")
        
        rclpy.shutdown()

    def _signal_handler(self, signum, frame):
        """Handle system signals for graceful shutdown."""
        self.running = False
        self.remoteControlService.close()


def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


if __name__ == "__main__":
    from omegaconf import OmegaConf

    config = OmegaConf.load("src/sairol_bridge/robot_bridge/example/T1/configs/T1.yaml")

    # Initialize ROS 2 node
    rclpy.init()
    t = threading.Thread(target=spin_in_background)
    t.start()
    nh = rclpy.create_node('t1_controller')
    rclpy.get_global_executor().add_node(nh)
    t1_node = T1ClientNode(nh, config)

    t1_node.start()
    if t.is_alive():
        t.join()

