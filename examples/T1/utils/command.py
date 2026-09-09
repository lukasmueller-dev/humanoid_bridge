from bridge_interface.msg import RobotCmd, MotorCmd

def init_Cmd_T1(robot_cmd: RobotCmd):
    motorCmds = [MotorCmd() for _ in range(23)]
    robot_cmd.motor_cmd = motorCmds
    robot_cmd.interpolation_order = 0.8
    robot_cmd.hold_position = False
    robot_cmd.duration = 0.002

    for i in range(23):
        robot_cmd.motor_cmd[i].q = 0.0
        robot_cmd.motor_cmd[i].dq = 0.0
        robot_cmd.motor_cmd[i].tau = 0.0
        robot_cmd.motor_cmd[i].kp = 0.0
        robot_cmd.motor_cmd[i].kd = 0.0




def create_prepare_cmd(robot_cmd: RobotCmd, cfg):
    init_Cmd_T1(robot_cmd)
    for i in range(23):
        robot_cmd.motor_cmd[i].kp = cfg["prepare"]["stiffness"][i]
        robot_cmd.motor_cmd[i].kd = cfg["prepare"]["damping"][i]
        robot_cmd.motor_cmd[i].q = cfg["prepare"]["default_qpos"][i]
    return robot_cmd


def create_first_frame_rl_cmd(robot_cmd: RobotCmd, cfg):
    init_Cmd_T1(robot_cmd)
    for i in range(23):
        robot_cmd.motor_cmd[i].kp = cfg["common"]["stiffness"][i]
        robot_cmd.motor_cmd[i].kd = cfg["common"]["damping"][i]
        robot_cmd.motor_cmd[i].q = cfg["common"]["default_qpos"][i]

    return robot_cmd


def create_stop_frame_rl_cmd(robot_cmd: RobotCmd, cfg):
    init_Cmd_T1(robot_cmd)
    for i in range(23):
        robot_cmd.motor_cmd[i].kp = 0.0
        robot_cmd.motor_cmd[i].kd = 0.0
        robot_cmd.motor_cmd[i].tau = 0.0

    return robot_cmd