///-------File system-------///

custop_teleop
	package with python file to run full keyboard control of drone.


drone_demo
	package with some auto misions for drone in python.


cvg_sim_gazebo
	package with all launch files, worlds, model drone, model sensors and config to amcl;
	contain src/include folders with code to autopilot for drone in the world


message_to_tf 
	package used to create a ros node, which transfers the ros topic /ground_truth/state to a /tf topic.


cvg_sim_msgs
	package contains message forms for the simulator.


gazebo_models
	package contains files(meshes, config, sdf) for my models, what I use in wold files.



///------Commands------///


--Open test world, and spawn model

roslaunch cvg_sim_gazebo ardrone_test_world.launch

--Open rviz  for this simulation

roslaunch cvg_sim_gazebo mybot_rviz.launch



//Storage the reading map from the open world

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


//Custom/auto control drone

--drive in keyboard
rosrun custom_teleop teleop_twist_keyboard.py

//drone_demo

-- for ex. fly in square 
rosrun drone_demo squae_move.py


//Take map of nodes
/graph of my transform tree
rosrun tf view_frames

or
/graph of my communication between nodes and topics
rosrun rqt_graph rqt_graph



