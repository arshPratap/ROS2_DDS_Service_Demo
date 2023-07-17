# Action_Demo (Work in Progress) 

## Prerequisites
- Install [ROS-2 Humble](https://docs.ros.org/en/humble/Installation.html)
- Install the necessary ROS-2 packages in [ardupilot_ros2](https://github.com/arshPratap/ardupilot_ros2) 
    - switch to the **ROS2action** branch
    - build the [ap_custom_services](https://github.com/arshPratap/ardupilot_ros2/tree/ROS2action/ap_custom_services) package
    - build the [at_client](https://github.com/arshPratap/ardupilot_ros2/tree/ROS2action/at_client) package
- Install [Micro-XRCE-Agent](https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#installing-the-agent-standalone)
- Install [Micro-XRCE-Client library](https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#installing-the-client-standalone)
- Install [Integration Services](https://integration-service.docs.eprosima.com/en/latest/installation_manual/installation.html) : 
    - Get system handles for [Fast-DDS](https://github.com/eProsima/FastDDS-SH)
    - Get system handles for [ROS 2](https://github.com/eProsima/ROS2-SH)
    - Install the integration service by running the following command : `colcon build --cmake-args -DMIX_ROS_PACKAGES="ap_custom_services"`
## Arming Motors Example (DDS Server)
### Terminal 1 (XRCE-Agent)
- Run XRCE Agent using the following command `cd /usr/local/bin && MicroXRCEAgent udp4 -p 2019`
### Terminal 2 (XRCE-Client)
- In the **Arming_DDS_Server** folder, compile the Server file as `g++ DDS_Action_Server.cpp -lmicrocdr -lmicroxrcedds_client`
- Run the Server script as  `./a.out 127.0.0.1 2019`
### Terminal 3 (Integration Service) (Help Needed)
- Source your ROS 2 installation 
- Source the installation of the ROS-2 packages present in **ardupilot_ros2** from your ROS 2 workspace
- Source Integration Services from your workspace 
- Run the service using the following command `DDS_Action_Server_IS_config.yaml` from the **Arming_DDS_Server** folder
### Terminal 4 (ROS-2)
- Source ROS-2 installation
- Source the installation of the ROS-2 packages present in **ardupilot_ros2** from your ROS 2 workspace 
- Run the ROS-2 client as `ros2 run at_client gm_action_client`
