
# 🤖 robot_bridge Overview

`robot_bridge` is a ROS 2 node for controlling Unitree robots (e.g., H1, G2).  
It receives motion commands, performs trajectory interpolation, and sends real-time motor control messages via `/lowcmd`.

---

## 🚀 Quick Start

```bash
ros2 launch robot_bridge robot_bridge_launch.py
This will:

Stop any existing publisher on /lowcmd (via release_node)

Start the control bridge node

🧩 Core Interfaces
Subscribers:

/lowstate: Robot state input

/robot_cmd: Target motion commands

/wirelesscontroller: Remote control input

Publisher:

/lowcmd: Realtime motor command output

Services:

/start_control

/stop_control

/ready_position_control

/zero_position_control

🔒 Safety Mechanisms (Key Features)
✅ 1. Connection Timeout Protection
If no /lowstate message is received for >0.1s, the controller shuts down automatically to prevent unsafe behavior.

✅ 2. Joint Velocity Limits
Every joint has a configured dq_limit.
If exceeded, an error is logged and the node shuts down for safety.

✅ 3. Torque Prediction-Based Gain Scaling
Before applying control, the node predicts resulting torque:

τ_predict = Kp * (q_cmd - q) + Kd * (dq_cmd - dq)
If this exceeds the joint's τ_limit, the controller solves a quadratic equation to scale gains:

scss
Kp ← Kp * x
Kd ← Kd * sqrt(x)
If no positive root exists, fallback values are used.

✅ 4. Auto-Fill Gains (Kp/Kd)
If incoming command has both gains = 0, default gains are filled to prevent instability.

✅ 5. Command and State Validation

All control values (q, dq, tau, kp, kd) are clamped to joint-safe limits

IMU values (roll, pitch, yaw, accel, gyro) are validated

NaN/Inf values are checked and blocked

✅ Recommended Usage
bash
ros2 launch robot_bridge robot_bridge_launch.py