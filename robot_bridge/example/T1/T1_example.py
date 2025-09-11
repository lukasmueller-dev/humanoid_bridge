import rclpy
import time
import numpy as np
# from utils.remote_control_service import RemoteControlService
from utils.policy import Policy
import yaml
from robot_bridge_py.robot_client import RobotClient, BoosterJoyButton
from enum import Enum


class RobotController:   
    def __init__(self, node, cfg):
        self.node = node  
        self.robot = RobotClient(node=self.node, robot_type="T1", num_dof=23, control_frequency=50.0, interpolation_order=0.8)
        self.timer = self.node.create_timer(0.02, self.step)  # 50 Hz control frequency
        
        # Initialize components
        # self.remoteControlService = RemoteControlService()
        self.policy = Policy(cfg=cfg)

        self.init_pos = np.array([float(cfg["common"]["default_qpos"][i]) for i in range(self.robot.num_dof)], dtype=np.float32)
        self.policy_kp = np.array([float(cfg["common"]["stiffness"][i]) for i in range(self.robot.num_dof)], dtype=np.float32)
        self.policy_kd = np.array([float(cfg["common"]["damping"][i]) for i in range(self.robot.num_dof)], dtype=np.float32)
        
        self.agent_started = False
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.vyaw_cmd = 0.0
        print("Please press\n\t \"LT + START\" to start control, \n\t \"LT + A\" to start inferrence, \n\t \"BACK\" to stop control, \n\t \"LB\" for ready position, \n\t \"RB\" for zero position, \n\t \"LT + BACK\" for emergency stop.")

    def step(self):
        self.check_state()
        if self.robot.control_started and self.agent_started:
            self.policy_step()  
            
    def policy_step(self):
        time_now = self.robot.time_count / 500.
        q_des = self.policy.inference(
            time_now=time_now,
            dof_pos=self.robot.q_pos,
            dof_vel=self.robot.q_vel,
            base_ang_vel=self.robot.base_ang_vel,
            projected_gravity=self.robot.projected_gravity,
            vx=self.vx_cmd,
            vy=self.vy_cmd,
            vyaw=self.vyaw_cmd,
            # vx=self.remoteControlService.get_vx_cmd(),
            # vy=self.remoteControlService.get_vy_cmd(),
            # vyaw=self.remoteControlService.get_vyaw_cmd(),
        )
        
        self.robot.send_cmd(q_target_pos=q_des, target_kp=self.policy_kp, target_kd=self.policy_kd)
        
    def check_state(self):
        self.robot.update_robot_state()

        if self.robot.joy_key is not None:
            if self.robot.joy_key.lt and self.robot.joy_key.a:
                if self.robot.control_started:
                    self.agent_started = True
                    self.vx_cmd = 0.0
                    self.vy_cmd = 0.0
                    self.vyaw_cmd = 0.0
                    self.node.get_logger().info("Agent started.")
                else:
                    self.node.get_logger().warn("Please start the control first by pressing LT + START.")

            if self.agent_started:
                if self.robot.joy_key.ly * -1 >= 1.0:
                    self.vx_cmd += 0.1
                elif self.robot.joy_key.ly * -1 <= -1.0:
                    self.vx_cmd -= 0.1
                elif self.robot.joy_key.lx * -1 >= 1.0:
                    self.vy_cmd += 0.1
                elif self.robot.joy_key.lx * -1 <= -1.0:
                    self.vy_cmd -= 0.1
                elif self.robot.joy_key.rx * -1 >= 1.0:
                    self.vyaw_cmd += 0.1
                elif self.robot.joy_key.rx * -1 <= -1.0:
                    self.vyaw_cmd -= 0.1
                if self.robot.joy_key.ls  or self.robot.joy_key.rs:
                    self.vx_cmd = 0.0
                    self.vy_cmd = 0.0
                    self.vyaw_cmd = 0.0

                self.vx_cmd = np.clip(self.vx_cmd, -0.5, 0.5)
                self.vy_cmd = np.clip(self.vy_cmd, -0.5, 0.5)
                self.vyaw_cmd = np.clip(self.vyaw_cmd, -0.4, 0.4)
                print(f"Velocity commands - vx: {self.vx_cmd}, vy: {self.vy_cmd}, vyaw: {self.vyaw_cmd}")
            else:
                self.vx_cmd = 0.0
                self.vy_cmd = 0.0
                self.vyaw_cmd = 0.0

            self.robot.joy_key = None  # Reset joy_key
            self.robot.joy_axes = np.zeros(6, dtype=np.float32)

        if not self.robot.control_started:
            self.agent_started = False


if __name__ == "__main__":
    rclpy.init()
    cfg_file = "src/humanoid_bridge/robot_bridge/example/T1/configs/T1.yaml"
    with open(cfg_file, "r", encoding="utf-8") as f:
        policy_cfg = yaml.load(f.read(), Loader=yaml.FullLoader)
    node = rclpy.create_node('robot_client_node')
    controller = RobotController(node, policy_cfg)
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()