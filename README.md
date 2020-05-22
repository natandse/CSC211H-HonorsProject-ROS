# CSC211H-HonorsProject-ROS
Developing a project involving the ROS robot and testing it in the ROS virtual environment

simple_navigation_goals must be placed in the src folder of an existing catkin_ws on an Ubuntu 16.04 machine with
ROS Kinetic installed. To build, navigate to the catkin_ws folder, run the command "catkin_make", and then run the
command "source ~/atkin_ws/devel/setup.bash".

To run and visualize the files, open a terminal and run the "roscore" command to initialize ROS, then open a new 
terminal and run "roslaunch turtlebot_stage turtlebot_in_stage.launch" to open the visualizer, then open another 
terminal and run "rosrun simple_navigation_goals move_on_command cmd_vel:=/cmd_vel_mux/input/navi" to run the 
move_on_command program and finally open another terminal and run "rostopic pub /move_command std_msgs/String "
followed by either forward, backward, left, or right in double quotes ("")
