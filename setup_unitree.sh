if [ "$USER" != "unitree" ]; then
    echo "Source /opt/ros/humble/setup.bash and set RMW_IMPLEMENTATION and CYCLONEDDS_URI"
    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                                <NetworkInterface name="enp4s0" priority="default" multicast="default" />
                            </Interfaces></General></Domain></CycloneDDS>'
fi

if [ -z "$ROS_DISTRO" ]; then
    echo "ROS_DISTRO is not set. Please source ros2"
fi