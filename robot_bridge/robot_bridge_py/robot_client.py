import rclpy
import time
from enum import Enum
import numpy as np
from bridge_interface.msg import RobotCmd, MotorCmd
from bridge_interface.srv import SetDefaultPosition

from std_srvs.srv import Trigger
from rclpy.node import Node
from rclpy.client import Client as ROSClient

class BoosterJoyButton:
    Button_X = 1 << 0      # buttons[0]
    Button_A = 1 << 1      # buttons[1]
    Button_B = 1 << 2      # buttons[2]
    Button_Y = 1 << 3      # buttons[3]
    Button_LB = 1 << 4     # buttons[4]
    Button_RB = 1 << 5     # buttons[5]
    Button_LT = 1 << 6     # buttons[6]
    Button_RT = 1 << 7     # buttons[7]
    Button_BACK = 1 << 8   # buttons[8]
    Button_START = 1 << 9  # buttons[9]
    Button_LAXES = 1 << 10  # buttons[10]
    Button_RAXES = 1 << 11  # buttons[11]


class RobotClient:
    def __init__(self, node, robot_type, num_dof, control_frequency, interpolation_order=0) -> None:
        self.node: Node = node
        self.robot_type = robot_type
        self.num_dof = num_dof
        self.time_count = 0
        self.control_frequency = control_frequency

        self.cmd = RobotCmd()
        self.cmd.motor_cmd = [MotorCmd() for _ in range(self.num_dof)]
        self.cmd.interpolation_order = interpolation_order
        self.cmd.hold_position = False
        self.cmd.duration = 1 / control_frequency

        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.projected_gravity = np.zeros(3, dtype=np.float32)
        self.rpy = np.zeros(3, dtype=np.float32)
        self._q_pos = np.zeros(num_dof, dtype=np.float32)
        self._q_vel = np.zeros(num_dof, dtype=np.float32)
        self._tau = np.zeros(num_dof, dtype=np.float32)

        self.q_target_pos = np.zeros(num_dof, dtype=np.float32)
        self.q_target_vel = np.zeros(num_dof, dtype=np.float32)
        self.target_tau = np.zeros(num_dof, dtype=np.float32)
        self.target_kp = np.zeros(num_dof, dtype=np.float32)
        self.target_kd = np.zeros(num_dof, dtype=np.float32)

        self._default_pos = np.array([0.0,  0.0,
                                     0.25, -1.4, 0.0, -0.5,
                                     0.25, 1.4, 0.0, 0.5,
                                     0.0,
                                     -0.1, 0.0, 0.0, 0.2, -0.1, 0.0,
                                     -0.1, 0.0, 0.0, 0.2, -0.1, 0.0,])
        self._default_kp =  np.array([5., 5.,
                                     40., 50., 20., 10.,
                                     40., 50., 20., 10.,
                                     100., 
                                     350., 350., 180., 350., 450., 450.,
                                     350., 350., 180., 350., 450., 450.])
        self._default_kd = np.array([0.1, 0.1,
                                    0.5, 1.5, 0.2, 0.2,
                                    0.5, 1.5, 0.2, 0.2,
                                    5.0,
                                    7.5, 7.5, 3., 5.5, 0.5, 0.5,
                                    7.5, 7.5, 3., 5.5, 0.5, 0.5])
        self._default_duration = 2.0

        if self.robot_type == "T1":
            from booster_interface.msg import LowState, RemoteControllerState
            state_topic_name = '/low_state' 
            joy_topic_name = '/remote_controller_state'
            low_state_handler = self._low_state_handler_booster
            joy_handler = self._joy_handler_booster
        else: 
            from unitree_go.msg import LowState
            state_topic_name = '/lowstate'
            joy_topic_name = '/joy'
            low_state_handler = self._low_state_handler_unitree
            joy_handler = self._joy_handler_unitree

        self.low_state_subscription = self.node.create_subscription(LowState, state_topic_name, low_state_handler, 1)
        self.joystick_subscription = self.node.create_subscription(RemoteControllerState, joy_topic_name, joy_handler, 1)

        self.low_cmd_publisher = self.node.create_publisher(RobotCmd, '/robot_cmd', 1)
        
        self.start_control_client = self.node.create_client(SetDefaultPosition, '/start_control')
        self.goto_zero_position_client = self.node.create_client(Trigger, '/zero_position_control')
        self.ready_position_client = self.node.create_client(Trigger, '/ready_position_control')
        self.stop_control_client = self.node.create_client(Trigger, '/stop_control')

        self.joy_key = None
        # self.joy_axes = np.zeros(6, dtype=np.float32)
        self.control_start_time = None
        self.control_started = False

    @property
    def q_pos(self):
        return self._q_pos

    @property
    def q_vel(self):
        return self._q_vel

    @property
    def tau(self):
        return self._tau
    
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

    def _low_state_handler_booster(self, low_state_msg):
        self.time_count += 1
        self.rpy[:] = low_state_msg.imu_state.rpy

        self.projected_gravity[:] = self._rotate_vector_inverse_rpy(
            low_state_msg.imu_state.rpy[0],
            low_state_msg.imu_state.rpy[1],
            low_state_msg.imu_state.rpy[2],
            np.array([0.0, 0.0, -1.0]),
        )
        self.base_ang_vel[:] = low_state_msg.imu_state.gyro

        for i, motor in enumerate(low_state_msg.motor_state_serial):
            self._q_pos[i] = motor.q
            self._q_vel[i] = motor.dq
            
    def update_robot_state(self):
        time_now = time.time()
        if self.control_start_time is not None and time_now > self.control_start_time:
            self.control_started = True
        else:
            self.control_started = False
            
    def _low_state_handler_unitree(self, low_state_msg):
        raise NotImplementedError("Unitree low state handler is not implemented yet.")
    
    def _joy_handler_booster(self, joy_msg):
        """
        Handle joystick messages for the Booster robot.
        """
        # buttons = np.array(joy_msg.buttons)
        # self.joy_axes = np.array(joy_msg.axes)
        # key = np.dot(buttons, 2 ** np.arange(buttons.size))

        time_now = time.time()

        # self.joy_key = None
        # if key == (BoosterJoyButton.Button_LT | BoosterJoyButton.Button_START):  # start: LT + START
        #     self.node.get_logger().info("Starting control...")
        #     if not self.control_started:
        #         future = self.init_control()
        #         self.control_start_time = time_now + self._default_duration
        #     return
        # elif key == BoosterJoyButton.Button_LB:  # ready position: LB
        #     self.node.get_logger().info("Ready position control...")
        #     if not self.control_started:
        #         self.goto_default_position()
        #         self.control_start_time = None
        #     else:
        #         self.node.get_logger().warn("Control already started, please stop the control first by pressing BACK.")
        #     return
        # elif key == BoosterJoyButton.Button_RB:  # zero position: RB
        #     self.node.get_logger().info("Zero position control...")
        #     if not self.control_started:
        #         self.goto_zero_position()
        #         self.control_start_time = None
        #     else:
        #         self.node.get_logger().warn("Control already started, please stop the control first by pressing BACK.")
        #     return
        # elif key == BoosterJoyButton.Button_BACK:  # stop: BACK
        #     self.node.get_logger().info("Stopping control...")
        #     future = self.stop_control()
        #     self.control_start_time = None
        #     return
        # elif key == (BoosterJoyButton.Button_LT | BoosterJoyButton.Button_BACK):
        #     self.control_start_time = None
        # else:
        #     # Set key only for unknown key combinations
        #     self.joy_key = key
        
        
        if joy_msg.lt and joy_msg.start: # start: LT + START
            self.node.get_logger().info("Starting control...")
            if not self.control_started:
                future = self.init_control()
                self.control_start_time = time_now + self._default_duration
            return
        elif joy_msg.lb:  # ready position: LB
            self.node.get_logger().info("Ready position control...")
            if not self.control_started:
                self.goto_default_position()
                self.control_start_time = None
            else:
                self.node.get_logger().warn("Control already started, please stop the control first by pressing BACK.")
            return
        elif joy_msg.rb:  # zero position: RB
            self.node.get_logger().info("Zero position control...")
            if not self.control_started:
                self.goto_zero_position()
                self.control_start_time = None
            else:
                self.node.get_logger().warn("Control already started, please stop the control first by pressing BACK.")
            return
        elif joy_msg.back:  # stop: BACK
            self.node.get_logger().info("Stopping control...")
            future = self.stop_control()
            self.control_start_time = None
            return
        elif joy_msg.lt and joy_msg.back:
            self.control_start_time = None
        else:
            # Set key only for unknown key combinations
            self.joy_key = joy_msg


    def _joy_handler_unitree(self, joy_msg):
        raise NotImplementedError("Unitree joystick handler is not implemented yet.")

    def send_cmd(self, q_target_pos=None, q_target_vel=None, target_tau=None, target_kp=None, target_kd=None):
        for i in range(self.num_dof):
            self.cmd.motor_cmd[i].q = float(q_target_pos[i]) if q_target_pos is not None else self._default_pos[i]
            self.cmd.motor_cmd[i].dq = float(q_target_vel[i]) if q_target_vel is not None else 0.0
            self.cmd.motor_cmd[i].tau = float(target_tau[i]) if target_tau is not None else 0.0
            self.cmd.motor_cmd[i].kp = float(target_kp[i]) if target_kp is not None else self._default_kp[i]
            self.cmd.motor_cmd[i].kd = float(target_kd[i]) if target_kd is not None else self._default_kd[i]
        self.low_cmd_publisher.publish(self.cmd)
        
    def set_default_cmd(self, default_pos=None, default_kp=None, default_kd=None):
        """
        Set the default command for the robot.
        """
        if default_pos is not None:
            self._default_pos = np.array(default_pos, dtype=np.float32)
        if default_kp is not None:
            self._default_kp = np.array(default_kp, dtype=np.float32)
        if default_kd is not None:
            self._default_kd = np.array(default_kd, dtype=np.float32)
        
    def init_control(self, default_pos=None):
        """
        Start the control loop by calling the init_control service.
        """
        if default_pos is None:
            default_pos = self._default_pos

        while not self.start_control_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('start_control service not available, waiting again...')
        
        request = SetDefaultPosition.Request()
        request.default_position = default_pos.tolist()
        future = self.start_control_client.call_async(request)
        return future.result()
    
    def stop_control(self):
        """
        Stop the control loop by calling the stop_control service.
        """        
        while not self.stop_control_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('stop_control service not available, waiting again...')

        request = Trigger.Request()
        future = self.stop_control_client.call_async(request)
        return future.result()
    
    def goto_default_position(self):
        """
        Send robot to default position by calling the ready_position_control service.
        """
        while not self.ready_position_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('ready_position_control service not available, waiting again...')

        request = Trigger.Request()
        future = self.ready_position_client.call_async(request)
        return future.result()
        
    def goto_zero_position(self):
        """
        Send robot to zero position by calling the zero_position_control service.
        """
        while not self.goto_zero_position_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('zero_position_control service not available, waiting again...')

        request = Trigger.Request()
        future = self.goto_zero_position_client.call_async(request)
        return future.result()