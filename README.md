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