# /active_gains: the kp and kd the bridge uses now, per joint, and where they come from.
# Transient-local, at 2 Hz and on each source change. Arrays follow joint, the bridge YAML's order.
string[] joint
float32[] kp
float32[] kd
string[] source   # config | override | robot_cmd | damp | limp
