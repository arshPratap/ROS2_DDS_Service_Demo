## Work In Progress Branch

# ROS2_DDS_Service_Demo

## Architecture

![WhatsApp Image 2023-06-01 at 3 24 41 PM](https://github.com/arshPratap/ROS2_DDS_Service_Demo/assets/62841337/cf2c1b9d-c5f3-43f9-b81c-432f1d9b46c2)

## Prerequisites
- Install [ROS-2 Humble](https://docs.ros.org/en/humble/Installation.html)
- Install [Micro-XRCE-Agent](https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#installing-the-agent-standalone)
- Install [Micro-XRCE-Client library](https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#installing-the-client-standalone)
- Install [Integration Services](https://integration-service.docs.eprosima.com/en/latest/installation_manual/installation.html) : 
  - Get system handles for [Fast-DDS](https://github.com/eProsima/FastDDS-SH)
  - Get system handles for [ROS 2](https://github.com/eProsima/ROS2-SH)
  - Install the integration service by running the following command : `colcon build --cmake-args -DBUILD_FASTDDS_EXAMPLES=ON -DMIX_ROS_PACKAGES="example_interfaces"`

## DDS Server
### Terminal 1 (XRCE-Agent)
- Run XRCE Agent using the following command `cd /usr/local/bin && MicroXRCEAgent udp4 -p 2019`
### Terminal 2 (XRCE-Client)
- In the DDS_Server folder, compile the Server file as `gcc Server.c -lmicrocdr -lmicroxrcedds_client`
- Run the Server script as  `./a.out 127.0.0.1 2019`
### Terminal 3 (Integration Service)
- Source ROS 2 installation 
- Source Integration Services from your workspace 
- Run the service using the following command `integration-service fastdds_server__addtwoints.yaml` from the DDS_Server folder
### Terminal 4 (ROS-2)
- Source ROS-2 installation
- Run the ROS-2 client as `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 17}"`
## ROS2 Server
### Terminal 1 (ROS-2)
- Source ROS-2 installation
- Run the ROS-2 server as `ros2 run demo_nodes_cpp add_two_ints_server`
### Terminal 2 (Integration Service)
- Source ROS 2 installation
- Source Integration Services from your workspace
- Run the service using the following command `integration-service ros2_server__addtwoints.yaml` from the ROS2_Server folder
### Terminal 3 (XRCE-Agent)
- Run XRCE Agent using the following command `cd /usr/local/bin && MicroXRCEAgent udp4 -p 2019`
### Terminal 4 (XRCE-Client)
- In the ROS2_Server folder, compile the Client file as `gcc Client.c -lmicrocdr -lmicroxrcedds_client`
- Run the Client script as  `./a.out 127.0.0.1 2019`
