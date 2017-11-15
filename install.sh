cp -R ./ros_is_awesome/catkin_ws/src/ros_is_awesome_* ~/catkin_ws/src/
cd ~/catkin_ws/ && catkin_make && source ~/catkin_ws/devel/setup.bash 
cd ~/catkin_ws/src
rm -rf ~/ros_is_awesome
