source /opt/ros/humble/setup.bash
source $HOME/sairol_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp4s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'