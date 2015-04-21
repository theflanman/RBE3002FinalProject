For simulation:
	launch gazebo:
		roslaunch turtlebot_gazebo turtlebot_world.launch
	launch gmapping:
		roslaunch turtlebot_gazebo gmapping_demo.launch
	run RViz:
		rosrun rviz rviz

things ToDo:
-create a navigation stack using a proper launch file
-get move_base to cooperate
 -test using RViz 2d nav points
-implement exploration algorithm as explained in http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.218.8895&rep=rep1&type=pdf