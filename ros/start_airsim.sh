gnome-terminal -x $SHELL -ic "term_title airsim_node; roscd airsim_ros_pkgs/launch; echo 'roslaunch airsim_node.launch'; bash"
gnome-terminal -x $SHELL -ic "term_title python; cd ~/Documents/AirSim/; bash"
gnome-terminal -x $SHELL -ic "term_title moveit; roscd hector_moveit_config/launch; echo 'roslaunch airsim.launch host:=asus1';bash"
gnome-terminal -x $SHELL -ic "term_title UE; cd ~/Documents/ue_proj; echo 'AirsimNH_1.4.0/LinuxNoEditor/AirSimNH.sh';bash"
cd ~/Documents/catkin_ws2/src/rqt_mypkg/scripts/airsim
python start_airsim.py
echo '/media/student/data55/catkin_ws2/src/rqt_mypkg/scripts/airsim$ python start_airsim.py'

