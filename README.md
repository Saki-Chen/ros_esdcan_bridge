# ros esdcan bridge

## dependency
* libntcan   
  http://www.esdshanghai.com/esd_download.html
* jsoncpp    
  https://github.com/open-source-parsers/jsoncpp

## build
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/Saki-Chen/ros_esdcan_bridge
cd ../..
catkin_make
```

## usage
```bash
source catkin_ws/devel/setup.bash
roslaunch ros_esdcan_bridge ros_esdcan_bridge.launch 
```

## configuration
```xml
<!-- one node per can channel -->
<node pkg="ros_esdcan_bridge" name="ros_esdcan_bridge" type="ros_esdcan_bridge" ns="esdcan_channel_1" output="screen">
    <!-- your channel id of can card -->
    <param name="can_circuit_id" value="1" />
    <!-- baud rate -->
    <param name="can_bit_rate" value="500000" />
    <!-- rx buffer size -->
    <param name="rx_queuesize" value="32" />
    <!-- tx buffer size -->
    <param name="tx_queuesize" value="32" />
</node>
```
This config would launch a node with follows topics:   
* /esdcan_channel_1/can_tx   
  This is publish by the node, which transfers all CAN message to ros topic.
* /esdcan_channel_1/can_rx
  This is subscript by the node, which transfers message published on the topic to CAN.