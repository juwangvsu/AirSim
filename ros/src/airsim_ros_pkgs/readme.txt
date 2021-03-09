3/6/21
	final verdict, depthimg_pc2 timestamp should be current ros time, not the time in airsim image
	response. this is so because the time gap between ros and ue game seems increase over time,
	but the time diff is the same for all sensor data msg obtained from ue engine. 

	depthimg lag behind robot pose/tf by about 0.7 second, modify code to comp the time
	stamp of depthimg,
	reason unknow.
	rosparam set /airsim_node/depth_lag 0.7
	time lag offset seems slowly inclrease. if lag behind, rot 30 deg, pc in rviz should rotate clockwise,
		if lag offset compensation too big, rot 30 will show counter clockwise rotation of pc in rvi)z

	airsim data msg is behind ros time by 0.3 seconds. when rviz display data, it use tf at ros time. 
	the published tf should have a time stamp at the current ros time or the sensor;s airsim timestamp?
	this seems to be ue game linux problem, restart airsim node did not change the time lag amount.
	the amount time diff: 
		(curr_time0-airsim_timestamp_to_ros(img_response.time_stamp)).toSec()
	a ugly fix: just add that time diff to the stamp on depthimg_pc2,
	aisim_depthimg_tf_time_diff.mp4

3/4/21
	max_range_ check each time before pub depthimg_pc2, also floor point option keep or remove

2/21/21
	airsim_node pose_cmd added.
		use moveToPositionAsync(), run 5 seconds.
	listen to /airsim_node/SimpleFlight/pose_cmd_world_frame
	test: rostopic pub /airsim_node/SimpleFlight/pose_cmd_world_frame 
		"...5,0,-2,0,0,0,1"
	Explorer_test.cpp 
		two threads: timer threads listen to goal topic and call go()
			main thread runs interactive
		listen to a target pose --- done
		call move_group to get a plan --- done
		 send traj points to airsim_node one by one by topic --- tbd
			pose_cmd_world_frame
	frame id:
		SimpleFlight: movelocal, movoto.py, odom, pose_cmd_world_frame
		world	: movegroup plan, rviz. trajectory
		conversion btw frame ids:
			trajectory -> to pose_cmd_world_frame

	/airsim_node/SimpleFlight/vel_cmd_body_frame

3/5/2021 velocity cmd test
	rostopic pub /airsim_node/SimpleFlight/vel_cmd_body_frame airsim_ros_pkgs/VelCmd "twist:
	  linear:
	    x: 0.0
	    y: 0.0
	    z: 0.0
	  angular:
	    x: 0.0
	    y: 0.0
	    z: 1.0" -r 10
rostopic pub /airsim_node/SimpleFlight/vel_cmd_body_frame airsim_ros_pkgs/VelCmd "twist:
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.0"

2/17/21
	
	added realsense node and yolo code 
        test steps:
	(1) roslaunch realsense2_camera rs_camera.launch color_width:=1280 color_height:=720
				--- launch realsense node, publish rgb, depth data and camera info. realsense-ros, sdk must be installed. 
	(2) python ros_yolo_detect.py --- this runs yolo on rgb feed, publish labels
				topic and detected images.
	(3) python listener.py       --- show detected yolo boxes

2/6/21 get_cloud_msg_from_depthimg()
	depth image also converted to point cloud, published
	at /airsim_node/SimpleFlight/depthimg_pc2
	clip at /airsim_node/max_range=20, set in launch file
	depth image and cloud also dumped, 
		~/.ros/depth_image.txt, 256x256 matrix, z value
		~/.ros/depth_cloud.asc, xyzrgb tubples
	frame_id="front_left_custom_body/static"
