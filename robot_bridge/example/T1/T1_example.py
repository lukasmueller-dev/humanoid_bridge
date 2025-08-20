import rclpy
import time
import numpy as np
from utils.remote_control_service import RemoteControlService
from utils.policy import Policy
import yaml
from robot_bridge_py.robot_client import JoyCmd, RobotClient
from enum import Enum


class RobotController:   
    def __init__(self, node, cfg):
        self.node = node  
        self.robot = RobotClient(node=self.node, robot_type="T1", num_dof=23, control_frequency=50.0, interpolation_order=0.8)
        self.timer = self.node.create_timer(0.02, self.step)  # 50 Hz control frequency
        
        # Initialize components
        self.remoteControlService = RemoteControlService()
        self.policy = Policy(cfg=cfg)

        self.init_pos = np.array([float(cfg["common"]["default_qpos"][i]) for i in range(self.robot.num_dof)], dtype=np.float32)
        self.policy_kp = np.array([float(cfg["common"]["stiffness"][i]) for i in range(self.robot.num_dof)], dtype=np.float32)
        self.policy_kd = np.array([float(cfg["common"]["damping"][i]) for i in range(self.robot.num_dof)], dtype=np.float32)
        
        self.control_started = False
        self.agent_started = False
        self.control_start_time = None
        
    def step(self):
        self.check_state()
        if self.control_started and self.agent_started:
            self.policy_step()  
            
    def policy_step(self):
        time_now = self.robot.time_count / 500.
        q_des = self.policy.inference(
            time_now=time_now,
            dof_pos=self.robot.q_pos,
            dof_vel=self.robot.q_vel,
            base_ang_vel=self.robot.base_ang_vel,
            projected_gravity=self.robot.projected_gravity,
            vx=self.remoteControlService.get_vx_cmd(),
            vy=self.remoteControlService.get_vy_cmd(),
            vyaw=self.remoteControlService.get_vyaw_cmd(),
        )
        
        self.robot.send_cmd(q_target_pos=q_des, target_kp=self.policy_kp, target_kd=self.policy_kd)
        
    def check_state(self):
        joy_state = self.robot.joy_state
        time_now = time.time()

        if joy_state == JoyCmd.INIT_CONTROL:
            future = self.robot.init_control(default_pos=self.init_pos)
            self.control_start_time = time_now + 2.0 
        elif joy_state == JoyCmd.STOP_CONTROL:
            future = self.robot.stop_control()
            self.control_start_time = None
        elif joy_state == JoyCmd.START_AGENT:
            self.agent_started = True
        elif joy_state == JoyCmd.DEFAULT_POSITION:
            self.robot.goto_default_position()
            self.control_start_time = None
        elif joy_state == JoyCmd.ZERO_POSITION:
            self.robot.goto_zero_position()
            self.control_start_time = None
            
        if self.control_start_time is not None and time_now > self.control_start_time:
            self.control_started = True
        else:
            self.control_started = False

        if joy_state != JoyCmd.EMPTY:
            self.robot.joy_state = JoyCmd.EMPTY

if __name__ == "__main__":
    rclpy.init()
    cfg_file = "src/sairol_bridge/robot_bridge/example/T1/configs/T1.yaml"
    with open(cfg_file, "r", encoding="utf-8") as f:
        policy_cfg = yaml.load(f.read(), Loader=yaml.FullLoader)
    node = rclpy.create_node('robot_client_node')
    controller = RobotController(node, policy_cfg)
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()