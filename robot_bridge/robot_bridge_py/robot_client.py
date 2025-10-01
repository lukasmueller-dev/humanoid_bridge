import rclpy
import time
from enum import Enum
import numpy as np
from bridge_interface.msg import RobotCmd, MotorCmd
from bridge_interface.srv import SetDefaultPosition


from std_srvs.srv import Trigger
from rclpy.node import Node
from rclpy.client import Client as ROSClient
import numpy as np
from scipy.spatial.transform import Rotation as R

def rpy_to_quat(rpy):
    r = R.from_euler('xyz', rpy)  
    quat = r.as_quat(scalar_first=True)  # return [w, x, y, z]
    return quat


class WirelessKey_H1_G1:
    KEY_R1     = 1 << 0
    KEY_L1     = 1 << 1
    KEY_START  = 1 << 2
    KEY_SELECT = 1 << 3
    KEY_R2     = 1 << 4
    KEY_L2     = 1 << 5
    KEY_A      = 1 << 8
    KEY_B      = 1 << 9
    KEY_X      = 1 << 10
    KEY_Y      = 1 << 11
    KEY_UP     = 1 << 12
    KEY_RIGHT  = 1 << 13
    KEY_DOWN   = 1 << 14
    KEY_LEFT   = 1 << 15


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
        
        self._q_pos = np.zeros(num_dof, dtype=np.float32)
        self._q_vel = np.zeros(num_dof, dtype=np.float32)
        self._tau = np.zeros(num_dof, dtype=np.float32)
        self._quat = np.zeros(4, dtype=np.float32)
        self._angular_velocity = np.zeros(3, dtype=np.float32)
        

        self._default_duration = 2.0
        self.joy_key = None
        
        if self.robot_type == "T1":
            from booster_interface.msg import LowState
            from booster_interface.msg import RemoteControllerState as JoyMsg
            state_topic_name = '/low_state' 
            joy_topic_name = '/remote_controller_state'
            low_state_handler = self._low_state_handler_booster
            joy_handler = self._joy_handler_booster

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
                                        350., 350., 180., 350., 300., 300.,
                                        350., 350., 180., 350., 300., 300.])
            self._default_kd = np.array([0.1, 0.1,
                                        0.5, 1.5, 0.2, 0.2,
                                        0.5, 1.5, 0.2, 0.2,
                                        5.0,
                                        7.5, 7.5, 3., 5.5, 0.5, 0.5,
                                        7.5, 7.5, 3., 5.5, 0.5, 0.5])
            self.key_count = 0
        elif self.robot_type == "G1": 
            from unitree_hg.msg import LowState
            from unitree_go.msg import WirelessController as JoyMsg
            state_topic_name = '/lowstate'
            joy_topic_name = '/wirelesscontroller'
            low_state_handler = self._low_state_handler_unitree
            joy_handler = self._joy_handler_unitree
            
            self._default_pos = np.array([-0.1,  0.0,  0.0,  0.3, -0.2, 0.0, 
                                          -0.1,  0.0,  0.0,  0.3, -0.2, 0.0,
                                           0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            
            self._default_kp =  np.array([100, 100, 100, 150, 40, 40, 
                                          100, 100, 100, 150, 40, 40,
                                          200, 200, 200,
                                          100, 100, 50, 50, 20, 20, 20,
                                          100, 100, 50, 50, 20, 20, 20])
            
            self._default_kd = np.array([6, 6, 6, 4, 2, 2, 
                                         6, 6, 6, 4, 2, 2,
                                         1, 1, 1, 
                                         2, 2, 2, 2, 1, 1, 1,
                                         2, 2, 2, 2, 1, 1, 1])
        elif self.robot_type == "H1": 
            from unitree_go.msg import LowState
            from unitree_go.msg import WirelessController as JoyMsg
            state_topic_name = '/lowstate'
            joy_topic_name = '/wirelesscontroller'
            low_state_handler = self._low_state_handler_unitree
            joy_handler = self._joy_handler_unitree
            
            self._default_pos = np.array([ 0.0, -0.1,  0.3,  
                                           0.0, -0.1,  0.3,
                                           0.0,  0.0,  0.0,
                                           0.0, -0.2, -0.2,
                                           0.0,  0.0,  0.0, 0.0,
                                           0.0,  0.0,  0.0, 0.0]) 
                                           
   
            self._default_kp =  np.array([150, 150, 200, 
                                          150, 150, 200, 
                                          300, 150, 150,
                                            0,  40,  40,
                                          100, 100, 50, 50, 
                                          100, 100, 50, 50])
            
            self._default_kd = np.array([2, 2, 4,
                                         2, 2, 4,
                                         3, 2, 2,
                                         0, 2, 2,
                                         2, 2, 2, 2,
                                         2, 2, 2, 2])

        self.low_state_subscription = self.node.create_subscription(LowState, state_topic_name, low_state_handler, 1)
        self.joystick_subscription = self.node.create_subscription(JoyMsg, joy_topic_name, joy_handler, 1)

        self.low_cmd_publisher = self.node.create_publisher(RobotCmd, '/robot_cmd', 1)
        
        self.start_control_client = self.node.create_client(SetDefaultPosition, '/start_control')
        self.goto_zero_position_client = self.node.create_client(Trigger, '/zero_position_control')
        self.ready_position_client = self.node.create_client(Trigger, '/ready_position_control')
        self.stop_control_client = self.node.create_client(Trigger, '/stop_control')

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
    
    @property
    def quat(self):
        return self._quat
    
    @property
    def angular_velocity(self):
        return self._angular_velocity

    def _low_state_handler_booster(self, low_state_msg):
        self.time_count += 1
        self._angular_velocity = low_state_msg.imu_state.gyro
        self._angular_acceleration = low_state_msg.imu_state.acc
        
        for i, motor in enumerate(low_state_msg.motor_state_serial):
            self._q_pos[i] = motor.q
            self._q_vel[i] = motor.dq
            self._tau[i] = motor.tau_est
        
        self._quat = rpy_to_quat(low_state_msg.imu_state.rpy)
        
    def _low_state_handler_unitree(self, low_state_msg):
        self.time_count += 1
        self._quat = low_state_msg.imu_state.quaternion
        self._angular_velocity = low_state_msg.imu_state.gyroscope
        self._angular_acceleration = low_state_msg.imu_state.accelerometer
        
        for i, motor in enumerate(low_state_msg.motor_state):
            if i < self._q_pos.shape[0]:
                self._q_pos[i] = motor.q
                self._q_vel[i] = motor.dq
                self._tau[i] = motor.tau_est
    
    def update_robot_state(self):
        time_now = time.time()
        if self.control_start_time is not None and time_now > self.control_start_time:
            self.control_started = True
        else:
            self.control_started = False
    
    def _joy_handler_booster(self, joy_msg):
        """
        Handle joystick messages for the Booster robot.
        """
        time_now = time.time()

        self.key_count = sum([
            joy_msg.a, joy_msg.b, joy_msg.x, joy_msg.y,
            joy_msg.lb, joy_msg.rb, joy_msg.lt, joy_msg.rt,
            joy_msg.ls, joy_msg.rs, joy_msg.back, joy_msg.start,
            joy_msg.hat_u, joy_msg.hat_d,
            joy_msg.hat_l, joy_msg.hat_r, joy_msg.hat_lu,
            joy_msg.hat_ld, joy_msg.hat_ru, joy_msg.hat_rd
        ])
    
        if joy_msg.lt and joy_msg.start and self.key_count == 2:  # start: LT + START
            self.node.get_logger().info("Starting control...")
            if not self.control_started:
                future = self.init_control()
                self.control_start_time = time_now + self._default_duration
            return
        elif joy_msg.lb and self.key_count == 1:  # ready position: LB
            self.node.get_logger().info("Ready position control...")
            if not self.control_started:
                self.goto_default_position()
                self.control_start_time = None
            else:
                self.node.get_logger().warn("Control already started, please stop the control first by pressing BACK.")
            return
        elif joy_msg.rb and self.key_count == 1:  # zero position: RB
            self.node.get_logger().info("Zero position control...")
            if not self.control_started:
                self.goto_zero_position()
                self.control_start_time = None
            else:
                self.node.get_logger().warn("Control already started, please stop the control first by pressing BACK.")
            return
        elif joy_msg.back and self.key_count == 1:  # stop: BACK
            self.node.get_logger().info("Stopping control...")
            future = self.stop_control()
            self.control_start_time = None
            return
        elif joy_msg.lt and joy_msg.back and self.key_count == 2:  # emergency stop: LT + BACK
            self.control_start_time = None
        else:
            # Set key only for unknown key combinations
            self.joy_key = joy_msg
  
    def _joy_handler_unitree(self, joy_msg):
        key = joy_msg.keys
        time_now = time.time()
        
        if (key & (WirelessKey_H1_G1.KEY_L2 | WirelessKey_H1_G1.KEY_START)) == (WirelessKey_H1_G1.KEY_L2 | WirelessKey_H1_G1.KEY_START):
            self.node.get_logger().info("Starting control...")
            if not self.control_started:
                future = self.init_control()
                self.control_start_time = time_now + self._default_duration
            return
        elif (key & (WirelessKey_H1_G1.KEY_L2 | WirelessKey_H1_G1.KEY_UP | WirelessKey_H1_G1.KEY_LEFT)) == (WirelessKey_H1_G1.KEY_L2 | WirelessKey_H1_G1.KEY_UP | WirelessKey_H1_G1.KEY_LEFT):
            self.node.get_logger().info("Stopping control...")
            future = self.stop_control()
            self.control_start_time = None
            return
        elif key & WirelessKey_H1_G1.KEY_L1:
            self.node.get_logger().info("Ready position control...")
            if not self.control_started:
                self.goto_default_position()
                self.control_start_time = None
            else:
                self.node.get_logger().warn("Control already started, please stop the control first by pressing BACK.")
            return
        elif key & WirelessKey_H1_G1.KEY_R1:
            self.node.get_logger().info("Zero position control...")
            if not self.control_started:
                self.goto_zero_position()
                self.control_start_time = None
            else:
                self.node.get_logger().warn("Control already started, please stop the control first by pressing BACK.")
            return  
        else:
            # Set key only for unknown key combinations
            self.joy_key = joy_msg
            
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