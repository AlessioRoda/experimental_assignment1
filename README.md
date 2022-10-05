# Cluedo simulation for the Experimental Robotics Lab course, Alessio Roda S4458313
The purpose of this repository is to create a simulation for the Clued game in which the robot moves in four different positions to get the required hints
to obtain the game solution. The hints are generated when the cluedo_link in the robot arm reaches the marker corresponding to the position that it reached; 
the marker is not reachable from the robot itself, but it's reachable with the arm. The received hints may generate complete and consistent hypotheses that are used as possible solutions for the cluedo game: after hgaving collected all the hints the robot goes in the center of the map (the Oracle room) and checks if one of the complete and consistent hypothesis corresponds to the solution; if the right solution is found the game ends, otherwhise the robot continues movieng in the four positions available.


## Description of the architecture

![cluedo](https://user-images.githubusercontent.com/48511957/142238407-b648df07-2806-474c-a22e-d787d1638970.jpg)

The architecture is divided into two packages:
* erl2: the main package of the architecture: it provides all the code for simulating the cluedo game and all the noes to interact with the ARMOR service and to manage the ROSPlan behaviour
* moveit_config: the package generated with the Moveit setup assistant for generating all the packages that allow to control the motion of the arm

Inside the erl2 folder you can find:
* action folder containing the Move.action file for generating the corresponding action message
* config folder containing the configuration files for the Rviz simulation and the motor controllers
* docs folder with the sphinx code documentation (to visualize use docs/_build/html/index.html)
* launch folder in which there are the launch files for running all the nodes of the simulation
* msg folder containing the ErlOracle.msg for defining the hint messages
* PDDL folder containing the domai.pdl and problem.pddl files that descrives respectively the domain and the problem of the cluedo simulaiton
* scripts folder containing the go_to_point.py script to implement a simple behavioiur for moving the the robot from a position to another in the simulation, the ontology_interface.py script to manage the queries for the ARMOR ontology and the reset_planner.py script that manages the ROSPlan behaviour, dispatching the problem until a solution is found. It also contains a sub folder with the MyArmor and Planner classes.
* src folder with the init.cpp file that allows to move the robotic armi in the initial configuration and the check_consistency.cpp, goToWaypoint.cpp and movearm.cpp files that represent the Action Interfaces nodes for performing the actions described in the PDDL doamin file. The folder also contains the simulaiton.cpp file that manages the simulaion node for generating the hints when a marker is reached with the arm 
* srv folder with all the service custom messages
* urdf folder with the robot.urdf description file for the simulation
* world folder containing the world description of the simulation
* cluedo_ontology.owl that describes the initial ontology
* launch_simulation.sh for launching all the launch files simultaneously

Concerning the aruco_ros and moveit_config packages only small changes have been performed to change the default Gazebo simulation environment, generated by the setup assistant, to the new simulation environment provided for the Cluedo game.

### Component diagram
![diagram drawio](https://user-images.githubusercontent.com/48511957/193786561-4bdc58c4-d293-4714-b183-c190a70d0ca8.png)


Here there's the UML diagram to better explain the behaviour of the architecture: the reset_planner component updates the knowledge base by adding all the informations regarding the PDDL problem, then, once the plan is generated and dispatched, the ROSPlan component provides to execute the nodes that corresponds to the PDDL actions described in the domain file (movearm, go_to_waypoint, movearm, update_ontology, check_consistency), thanks to the ROSPlan Action Interface. Regarding the goToWaypoint component, it generates an action goal with the x, y and orientation coordinates of the position to reach, that depend on the corresponding waypoint to reach in the generated plan; then it sends the goal on topic go_to_point_action and waits untill the action is completed.
The action is executed by the go_to_point module, that implements a simple behaviour to move the robot from its actual position to the goal position, first by rotating it to the correct direction of the target, then by sending a linear velocity proportional to the distance to travel. The movearm module moves the robotic arm from its actual position to a defined positon that allows to reach the marker, then it retutns in its default position. The updateOntology and checkConistency components send service messages requests on topic /update_request and /consitency_request that are received from the oracle_interface node. This node provides the following behaviour:
* updates the cluedo ontology with the people, the weapons and the places of the cluedo simulation
* receives a hint as /oracle_hint service message each time the cleudo_link of the robotic arm reaches one of the markers in the simulation 
* adds the received hints to the ontolology each time it receives a request from topic /update_request
* when the /consistency_request request is received, it queries all the complete and consistent hypotheses in the ontology and checks if one of them corresposnds to the solution of the game, that is received as a service response on topic /oracle_solution. Finally it also sends a service message on topic /reset_planning with the parameter finished equal to True or False in case the simualtion was found or not.

The simulaiton component was already provided for the simulation (here the link to the original repository https://github.com/CarmineD8/erl2/), it manages the creations of the hints when the cluedo_link enters in one of the markers and provide the solution of the cluedo game.
Finally, the init component moves the robotic armi in the correct initial position when the simualtion starts.

Here there's the rqt_graph to have a better idea of how the nodes interact between eachothers 

![rosgraph](https://user-images.githubusercontent.com/48511957/193853343-7b11a373-0718-4108-93f2-c6eca8099c01.png)

### Flowchart 
![flowchart drawio](https://user-images.githubusercontent.com/48511957/193857081-69e2ce0b-6a7c-4b8c-8e15-7d21e12266ae.png)

The general behaviou of the architecture can be described as the flowchart: at the beginning a plan is generated from the doamin and problem file, then it is dispatched with the nodes previously described that allow to perform the actions of the plan and finally, if one of the complete and consistent hypothesis found in the check_consitency action is correct, the game ends, otherwise the plan is generated again with the same informations regarding the domain and the porblem and the plan in disaptched.

### Temporal diagram
![temporal_diagram](https://user-images.githubusercontent.com/48511957/194004636-da1eddef-41b1-4404-b962-80a0ad12c4ba.png)

As you can see in the diagram, the first step is to initialize the plan, then the first action that is executed is the go_to_waypoint one, to move the robot from the oracle room, where the simulation starts, to one of the four rooms. Notice that, since in the problem file the doistance between the rooms is different in order to make the simulaiton more realistic, the robot will move from a room to another by following the shortest path. Once the room is reached, the robot performs the movearm action, to reach the marker with the arm; after that it moves to another room and repeat the same procedure until every room is visted. After that the marker of the fourth room is reached, the robot moves to the oracle room, then performs the update_ontology operation to ass all the new hints to the ontology and then it also performs the checK_consistency action to find complete and consistent hypotheses and use them as possible solutions. If the solution is found the game ends, otherwise the simulation starts again from the initialization phase.

## How to run the code

This project was developed and tested on Ubuntu 20.04 with ROS noetic, in order to install all the necessary dependencies make sure you have installed in your system:

* ROSPlan (here the link: https://github.com/KCL-Planning/ROSPlan)
* ARMOR (https://github.com/EmaroLab/armor)
* Moveit

After that, clone the repository in your ros workspace, enter in it and move to the second_assignment branch with the git command 

```
git checkout second_assignment
```
compile the repository with 

```
catkin_make
```
Then the best way to run the code is to go inside erl2 folder, then launch the simulation by running in your terminal

```
./launch_simulation.sh
```

### Run the nodes separately

If you want to have a view of all the nodes that are running with all the logs that they print, you can run them separately; in order to do so, please follow the commands above

```
roslaunch erl2 assignment.launch
```
```
rosrun armor execute it.emarolab.armor.ARMORMainService
```
```
rosrun erl2 init
```
```
rosrun erl2 ontology_interface.py _ontology:=(your_ros_workspace)/exp_assignment1/erl2/cluedo_ontology.owl
```
```
rosrun erl2 go_to_point.py
```
```
rosrun erl2 simulation
```
Then I recomend to use the planning.launch to manage the ROSPlan part by launching 
```
roslaunch erl2 planning.launch
```

## Simulation recording
Here you can find the video recording about the whole simulation https://unigeit-my.sharepoint.com/:v:/g/personal/s4458313_studenti_unige_it/Ed1zH-aaicZDuqy1-qnX81QB9XzlwxRgDBhAbw1-LA275A?e=1GZrCt (I recommend to skip patrs of the video, since the simulation is quite long). Above there are some screenshots from the Gazebo and Rviz simulation.

![rviz](https://user-images.githubusercontent.com/48511957/194016679-724f04ae-2233-4410-aab1-181a5abd1afb.png)

![gazebo](https://user-images.githubusercontent.com/48511957/194016837-2c03bf00-d3d3-44d4-bdec-a47cd8b7f8b6.png)




