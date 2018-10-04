///-------File system-------///

custop_teleop
	package with python file to run keyboard control of dron


drone_demo
	package with some auto misions to drone 


cvg_sim_gazebo
	package with all launch files, worlds, model drone, model sensors and config to amcl


message_to_tf 
	package used to create a ros node, which transfers the ros topic /ground_truth/state to a /tf topic.


cvg_sim_msgs
	package contains message forms for the simulator.




///------Commands------///


--Open test world, and spawn model

roslaunch cvg_sim_gazebo ardrone_test_world.launch

--Open rviz for this simulation

roslaunch cvg_sim_gazebo mybot_rviz.launch



//Storage map 

rosrun gmapping slam_gmapping _odom_frame:=nav

roslaunch cvg_sim_gazebo gmapping_demo.launch

--Save map in .yaml, to open in navigation

rosrun map_server map_saver -f ~/path



//Navigation on map

roslaunch cvg_sim_gazebo ardrone_test_world.launch

--start build map 

roslaunch cvg_sim_gazebo amcl_demo.launch

--Open rviz to see map, and navigate drone on the map 

roslaunch cvg_sim_gazebo mybot_rviz.launch


//Custom_teleop

--drive in keyboard
rosrun custom_teleop teleop_twist_keyboard.py

//drone_demo

-- for ex. fly in square 
rosrun drone_demo squae_move.py


//Take map of nodes

rosrun tf view_frames

or

rosrun rqt_graph rqt_graph



