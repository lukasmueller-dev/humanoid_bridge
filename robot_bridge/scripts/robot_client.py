import os
import sys
import time
import threading
import logging
import rclpy
from rclpy.node import Node
import numpy as np
from bridge_interface.msg import RobotCmd, MotorCmd

class RobotClient:
    def __init__(self, robot_type, num_dof, nh_) -> None:
        self.nh_ = nh_
        self.robot_type = robot_type
        self.num_dof = num_dof
        self.count = 0


        self.cmd = RobotCmd()
        motor_cmds = [MotorCmd() for _ in range(self.num_dof)]
        self.cmd.motor_cmd = motor_cmds
        self.cmd.interpolation_order = 0.8
        self.cmd.hold_position = False
        self.cmd.duration = 0.002

        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.projected_gravity = np.zeros(3, dtype=np.float32)
        self.rpy = np.zeros(3, dtype=np.float32)
        self.dof_pos = np.zeros(num_dof, dtype=np.float32)
        self.dof_vel = np.zeros(num_dof, dtype=np.float32)
        self.dof_tau = np.zeros(num_dof, dtype=np.float32)

        self.dof_target_pos = np.zeros(num_dof, dtype=np.float32)
        self.dof_target_vel = np.zeros(num_dof, dtype=np.float32)
        self.dof_target_tau = np.zeros(num_dof, dtype=np.float32)
        self.dof_target_kp = np.zeros(num_dof, dtype=np.float32)
        self.dof_target_kd = np.zeros(num_dof, dtype=np.float32)

        if self.robot_type == "T1":
            from booster_interface.msg import LowState 
            
            self.low_state_subscription = self.nh_.create_subscription(
                LowState,
                '/low_state',
                self._low_state_handler,
                1
            )
        else: 
            from unitree_go.msg import LowState

            self.low_state_subscription = self.nh_.create_subscription(
                LowState,
                '/lowstate',
                self._low_state_handler,
                1
            )

        self.low_cmd_publisher = self.nh_.create_publisher(
            RobotCmd,
            '/robot_cmd',
            1
        )
        
        # 在单独的线程中开始spinning
        self.spin_thread = threading.Thread(target=self.spin, daemon=True)
        self.spin_thread.start()

    def spin(self):
        """
        Start spinning the node handler to process callbacks
        """
        try:
            rclpy.spin(self.nh_)
        except KeyboardInterrupt:
            pass
        finally:
            self.nh_.destroy_node()

    def _rotate_vector_inverse_rpy(self, roll, pitch, yaw, vector):
        """
        Rotate a vector by the inverse of the given roll, pitch, and yaw angles.

        Parameters:
        roll (float): The roll angle in radians.
        pitch (float): The pitch angle in radians.
        yaw (float): The yaw angle in radians.
        vector (np.ndarray): The 3D vector to be rotated.

        Returns:
        np.ndarray: The rotated 3D vector.
        """
        R_x = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        return (R_z @ R_y @ R_x).T @ vector


    def _low_state_handler(self, low_state_msg):
        if self.robot_type == "T1":
            self.count += 1
            self.rpy[:] = low_state_msg.imu_state.rpy

            self.projected_gravity[:] = self._rotate_vector_inverse_rpy(
                low_state_msg.imu_state.rpy[0],
                low_state_msg.imu_state.rpy[1],
                low_state_msg.imu_state.rpy[2],
                np.array([0.0, 0.0, -1.0]),
            )
            self.base_ang_vel[:] = low_state_msg.imu_state.gyro

            for i, motor in enumerate(low_state_msg.motor_state_serial):
                if i < self.num_dof:
                    self.dof_pos[i] = motor.q
                    self.dof_vel[i] = motor.dq
        else:
            pass



    def _send_cmd(self, dof_target_pos, dof_target_vel, dof_target_tau, dof_target_kp, dof_target_kd):

        for i in range(self.num_dof):
            self.cmd.motor_cmd[i].q = float(dof_target_pos[i])
            self.cmd.motor_cmd[i].dq = float(dof_target_vel[i])
            self.cmd.motor_cmd[i].tau = float(dof_target_tau[i])
            self.cmd.motor_cmd[i].kp = float(dof_target_kp[i])
            self.cmd.motor_cmd[i].kd = float(dof_target_kd[i])
        self.low_cmd_publisher.publish(self.cmd)
