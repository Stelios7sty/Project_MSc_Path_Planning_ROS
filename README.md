# Project_MSc_Path_Planning_ROS

# F1/10 - Path Planning Algorithms
Path Planning algorithms for autonomous racing using F1/10 simulation and ROS

## Instructions how to download and use the code:

  1. First download ROS noetic (not melodic). 
  2. Download F1Tenth-Simulation from that link and follow the instractions:  https://github.com/f1tenth/f1tenth_simulator   
  3. Git clone this repository. 
  4. Open a terminal and run the following command -> ***roslaunch f1tenth_simulator simulator.launch*** *for pure pursuit* and ***roslaunch f1tenth_simulator ftgmethod.launch*** *for follow the gap algorithm*
  5. Once the rviz is ready, open another terminal and run the following command: ->  ***rosrun f1tenth_simulation <name of the algorithm you want to run>***

## File Description:
  * ppnode.py -> pure pursuit algorithm 
  * follow_the_gap.py -> follow the gap for obstacle avoidance algorithm 
  * safety.py -> safety commands for the vehicle.
  * pathNode.py -> display the path for the pure pursuit algorithm
  
### Notes:
* *Remember to make every file executable by using 'chmod +x <file_name>'*
* *If you want to test the Follow The Gap algorithm change the levine.yalm map from the simulator.launch to levine_blockes.yalm, the opposite applies for the pure pursuit algorithm*
* *Dont forget to source the path in the terminal and start roscore -> 'source devel/setup.bash'*
