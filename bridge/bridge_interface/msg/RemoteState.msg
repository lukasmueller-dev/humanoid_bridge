# The operator's remote, decoded by the bridge. Names as in the bridge YAML's estop combos.
bool connected      # G1, Go2: the robot reports a remote in this message; T1: a gamepad message has arrived since the bridge started
string[] pressed    # buttons held now
float32 lx          # sticks, -1 to 1, as the robot reports them; always 0 on the T1 (the bridge does not decode its sticks)
float32 ly
float32 rx
float32 ry
