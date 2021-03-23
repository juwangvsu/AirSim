3/23   test pull
2/8/21 start_airsim.sh
	ui: explorer, trajectory node
	    
2/7/21 

	now test if movegroup can take depthimg_pc2 directly, or it need to 
	be a specifi frame_id
	also test if we can simple combine the pointcloud in airsim_node and 
	publish the combined point cloud as the original lidar pc2.
	change the name of the original lidar pc2 to a different topic
	so we will now pub 3 pc2.

	movegroup octomap works with depthimg_pc2. only 
	need to change pc2 topic in the yaml files
	see catkin_ws2/src/hector-moveit/src/hector_moveit_config
	-sensors_airsim_depthimg.yaml
	--sensors_airsim.yaml
	: and make sure valid tf exist to the frame_id of the depthimg_pc2. 

	depthimg_pc2 fail to handle glass window. lidar seems ok.

	bugs: moveit map quickly grow too large for outdoor game,
	use all memory and crash computer.
	to investigate the effect of setting file on map, such ans max range, resolution etc.
	data5/ue_proj/Blocks/Binaries/Linux/Blocks, blueprint scene, rotate -90 
	facing wall, map good. rotate back to -15 degree map start to outof 
	control. 590 mb. possible cause: ?? free space also represented in map is reason?

	fix:
		this is fixed by clip the point cloud at airsim_node when
	it publish. airsim_node now use a param max_range to do so. that way we
	don't need to dig deep into moveit hack.

----------------------------------------------------------------------
2/5/21 

	lidar sensor position is changed to 0,0,0.1 so it allow
	us to check if the data is aligned with depthimg_pc2
fixed: (1)
	depthimg_pc2 size fixed.
	settings.json image width and heigh must be the same
	to correctly calculate point cloud data. this is
	so due to the get_cloud_msg_from_depthimg
	camera position affect the pc2 calculation also.
	left camera setting now 256x256

	(2)	

	depthimg_pc2 frame_id was Simpleflight, need to be change to
	the camera frame id. otherwise the pointcloud is not displayed 
	aligned in rviz and no good to create map
	left camera position "X": 0.05, "Y": -0.06, "Z": 0.0

	camera pc2 frame_id should be further changed to a camera link that
	match the camera position. Now use SimpleFlight/front_left_custom_static
	this work and the lidar and depthimg now align perfectly.
	the tf from SimpleFlight/odom_local_ned to front_left_custom_body/static
	is statically published by airsim_node, use camera position data in
	settings.json

-------------------------------------------------------- 
2/4/21 convert depth image of front_left_custom to pc2

issue: pc2 pubished @ /air.../depthimg_pc2
	the size is wrong, see dumped data depthimg_cloud.txt
	compare to lidar.txt
	depthimg_pc2 size fixed.

--------------------------------------------
quick launch tips:

cd ~/Documents/catkin_ws2/src/rqt_mypkg/scripts/airsim
python start_airsim.py

copy settings/settings_sensor.json to ~/Documents/AirSim/
use the same setting on the machine that runs the game
launch airsim_node.launch
launch rviz.launch
cd /home/student/Documents/catkin_ws2/src/hector-moveit/src/hector_moveit_config/launch; roslaunch airsim.launch

--------------debug cmds ----------------
rosservice call /clear_octomap 
rostopic hz /move_group/monitored_planning_scene

-----------other tips -----------------
pyuic4 airsimui.ui > airsimui.py
