# Lidar-Odometry
Capstone Project

## Dependencies
- [Catkin Command Line Tools](https://catkin-tools.readthedocs.io) (for building packages)
    ```
    sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'

    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

    sudo apt-get update
    sudo apt-get install -y python3-catkin-tools
    ```
- [ROS](http://wiki.ros.org/ROS/Installation) (for Noetic)
    ```
    sudo apt-get install -y ros-noetic-navigation ros-noetic-robot-localization ros-noetic-robot-state-publisher
    ```
- [gtsam](https://gtsam.org/get_started/) (Georgia Tech Smoothing and Mapping library)
    ```
    sudo add-apt-repository ppa:borglab/gtsam-release-4.0
    sudo apt install -y libgtsam-dev libgtsam-unstable-dev
    ```
- [Velodyne](http://wiki.ros.org/velodyne)
    ```
    sudo apt-get install -y ros-noetic-velodyne
    ```
- Update all dependecies on your workspace folder:
    ```
    cd ~/catkin_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
## Build
- Go to your catkin workspace:
    ```
    cd ~/catkin_ws
    ```
- [Xsens ROS MTi Driver](http://wiki.ros.org/xsens_mti_driver)
    ```
    pushd src/xsens_ros_mti_driver/lib/xspublic && make && popd
    catkin build xsens_mti_driver
    ```
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
    ```
    catkin build lio_sam --mem-limit 50% -j 1
    ```
- Lidar Odometry (Custom launch files)
    ```
    catkin build lidar_odometry
    ```
## Configure
- Parameters configuration files: src/Lidar-Odometry/config/params.yaml
- Launch files: src/Lidar-Odometry/launch
### Tutorials
- Velodyne VLP-16 tutorial: http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16
- LIO-SAM tutorials: https://github.com/TixiaoShan/LIO-SAM/blob/master/README.md
- LIO-SAM GPS intergration: https://github.com/TixiaoShan/LIO-SAM/issues/312

## Run packages
- Source workspace:
    ```
    source devel/setup.zsh
    ```
- Launch the Xsens MTi and Velodyne drivers:
    ```
    roslaunch lidar_odometry sensors.launch
    ```
- Launch LIO-SAM:
    ```
    roslaunch lidar_odometry lio_sam.launch
    ```


    