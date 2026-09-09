"""The camera wire contract. Both halves of this package read it, so the server
and the client cannot drift apart.

ZMQ REQ/REP: the client sends ``REQUEST``, the server answers three parts,
``[rgb_jpeg, ir_jpeg, depth_uint16]``. Only part 0 carries a frame; parts 1 and
2 stay empty so a richer [rgb, ir, depth] server can replace this one without a
client change. A server that failed to grab answers with all three parts empty,
which the client turns into a timeout rather than a stale frame.
"""

# The Jetson on the robot's internal network.
DEFAULT_HOST = "192.168.123.164"
DEFAULT_PORT = 5556

# The HTTP preview, one above the ZMQ port.
DEFAULT_PREVIEW_PORT = 5557

# (width, height) the server captures and fits to before encoding. A caller that
# needs a particular shape — a policy's input size — passes its own.
DEFAULT_SIZE = (640, 480)

REQUEST = b"get"
