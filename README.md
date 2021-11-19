# Cluedo simulation for the Experimental Robotics Lab course, Alessio Roda S4458313

The purpose of this repository is to generate a simulation for the Cluedo game, in which the robot moves from a place to another, asks hints to an oracle and tries to find a solution of the game. In order to do this, all the information about the scene and the hints perceived from the oracle are utilized to update an ARMOR ontology, with which the robot can reason to generate a solution for the game.

## Description of the architecture

![cluedo](https://user-images.githubusercontent.com/48511957/142238407-b648df07-2806-474c-a22e-d787d1638970.jpg)

The whole architecture wants to represent the steps of a cluedo game: there's a scene containing some places, people and weapons; once the game starts a random solution containing a person (the killer in the cluedo game), a place (place of the crime) and a weapon is generated. The robot starts in the center of the map, the oracle room, then it moves randomly for the scene to find some hints that can help him to find the solution; each time the robot enters in a new place a hint is provided from an oracle, then when the robot has collected some appropriated hints it tries to generate a solution by going in the oracle room and formally pronouncing a sentence that defines the solution, indicating the name of the killer, the place and the utilized weapon.

In order to implement this architecture tin the package you can find four folders:

* docs
* launch
* scripts
* srv

The first one contains the Sphinx documentation where you can find a deptailed explanation about all the code, you can also read it by accessing to this link: https://alessioroda.github.io/experimental_assignment1/ 

The launch folder contains two different launch files: sim.launch and sim_smach.launch; each one of them is used to launch all the nodes of the cluedo simulation, but the second one can also show the the smach FSM (if you have installed the visualizer).

The srv folder contains the service custom messages: AskHint that returns thre arrays for the people, the weapons and the places collected in the hint and the ID of the hint, Move that sends the actual position and the final position to reached expressed in x and y coordinate, then returns a boolean to confirm if the 
target is reached and Solution that sends three string variables to define the person, the place and the weapon of the solution and returns a boolean to confirm.

Finally the scripts folder contains all the nodes of the architecture, in particular:


1) state_machine that is the "main" of the architecture, it provides a FSM with smach (here the link for further informations about it http://wiki.ros.org/smach), where the robot can switch between three different steps: 

* MOVE
* ENTER_ROOM
* SOLUTION 

![smach1](https://user-images.githubusercontent.com/48511957/142251464-eeb5acc4-5c36-4e0d-852d-eeb6d500b493.png)

In the first step the robot performs the motion from the actual position to another random place in the scene (all the rooms can provide hints, except to the Oracle_Room) by sending a Move custom message to the go_to_point node, that provides to simulate the motion.

![smatch2](https://user-images.githubusercontent.com/48511957/142331999-ce49c793-134a-492a-90c9-a34e732cc33e.png)

The ENTER_ROOM state is triggered after that the motion is performed, in this case the robot receives a hint from the oracle node via AskHint custom message, then uses the MyArmor class methods to access to the ontology to upload the hint as a new hypothesis and to check if there are complete and consistent hypothesis. In the case in which there are, the robot will move to the Oracle_Room in the MOVE state, otherwhise it will continue moving randomly in the scene.

![smatch3](https://user-images.githubusercontent.com/48511957/142332724-ecb558a8-ca57-42de-a146-f248f0414f39.png)

Finally the SOLUTION state is reached when the robot enters in the Oracle_Room; in this case the robot gets the person, the place and the weapon corresponding to the complete and consistent hypothesis and sends them to the oracle node via Solution custom message. If the solution is correct the game finishes, otherwhise it continue by returning in the MOVE state.

Inside the scripts folder there are also two classes:

* MyArmor 
* Place

The first one is implemented to use in a more comfortable way the ARMOR commands since the class provides some methods that already compile and send the message to ARMOR (for further explanations please have a look to the Sphinx documentation), while the second one is a simple way to define a place as an object with a name and its x and y coordinates.

2) oracle that simulate the oracle in the cluedo game: it generates a solution when the node is launched, then it generates random hints when receives the request from the state_machine node and checks if the solution provided by the state_machine node is correct or not. In order to generate the hints it sorts a random number of elemnt between the people, the weapons and the places of the scene and associate each hint with an ID. Since the number of element is totally random, the hints generated can be incompleted (without at least one between PERSON, PLACE and WEAPON) or inconsistent (complete but with more than one between PERSON, PLACE and WEAPON). 

3) go_to_point is a very simple node that simulates the movememnt of the robot from a place to another, it receives from the state_machine node the satring position of the robot and the target position to reach, then it computes the euclidean distance between the coordiantes of the two positions and waits for an amount of time proportional to that distance, after that it notifies the state_machine node that the new position has beeen reached. 

In the workspace there's also the cluedo_ontology.owl file, that is the inintial scene for the game. The scene at the beginning is empty, with just the three classes PERSON, PLACE and WEAPON defined, so that at the beginiing of the game the people, the places and the weapons are associated to the corresponding class.
Finally at the end of the game a file solution_cluedo_game is generated, which contains the the people, the places and the weapons of the scene and the correct hypothesis that represent the solution.

To better describe the behaviour of the architecture here there are the component diagram and temporal diagram:

### Component diagram

![UML_diagram](https://user-images.githubusercontent.com/48511957/142516652-f87dcadd-5d02-47fa-ad94-964be49b00d0.jpg)

Here it's possible to notice that the state_machine node is the "heart" of the entire architecture, in particular: 

* Comunicates with the go_to_point node by sending the position to reache (x and y coordinates of the actual position and the position to reach) and receiving a feedback (boolen)
* Comunicates with the oracle node by sending a hint request and receiving the hint(what[ ], where[ ], who[ ] arrays containig the elements of the hint and ID of the hint), then it also sends a possible solution (what, where, who strings) and receives a feedback if it's correct or not (boolean)
* Interacts with the MyArmor class in order to comunicate with the Update Ontology: it uses the methods of the class to perform operations and receives the ARMOR response

The MyArmor class comunicate with the Update Ontology node by sending an ARMOR message and receiving the ARMOR response that passes to the state_machine node.
Finally the state_machine nodes generates Place objects with the Place class, by defining the x and y coordinates of a place and its name.

### Temporal diagram

![temporal_diagram](https://user-images.githubusercontent.com/48511957/142653957-c1809d9b-c01f-4ddd-b8bb-db8900da5794.jpg)

## How to run the code
First if you want to run this code you must have the ARMOR package installed in your workspace (otherwhise you ca download from here https://github.com/EmaroLab/armor), then download this folder in your ros workspace and compile it with 

```
catkin_make
```

Then open a terminal and do 

```
roscore &
```

```
rosrun armor execute it.emarolab.armor.ARMORMainService
```

Finally, on the basis of your preference, consider that there are different ways to run the code:

### Using sim.launch

In this case you just have to open another terminal and paste 

```
roslaunch experimental_assignment1 sim.launch
```
In this case you will see all the logs of the state_machine node, but the ones of the other nodes of the simulation will not appear in order to make the simualtion easier to follow on the terminal.

### Using sim_smach.launch

Do

```
roslaunch experimental_assignment1 sim_smach.launch
```

Here as befor all the nodes are launched and just state_machine plots the logs, in this case you can also see the simulation switching from a state to another in the FSM thanks to the smach viewer.

### Running all the nodes separatly

If you want to see al the logs generated by the nodes in order to have a deeper idea of what is happening in the simualtion, then run the nodes in different terminals by following this order:

```
rosrun experimental_assignment1 oracle.py
```

```
rosrun experimental_assignment1 go_to_point.py
```

```
rosrun experimental_assignment1 state_machine.py
```

Here you can also find a video showing the simulation: 


## System features and limitations

In according to what has been reported for the description of the architecture, it's important to notice that the purpose of the cluedo simulation is to stress the usage of ARMOR: in this sense the technical choices for the genreation of the hints and the mechanisms to deriver the solution are deliberately made for asking continusly the oracle for hints untill one of them is not the correct solution. For this reason the oracle can generate hints that often are incomplete, envenmore when the oracle gets the solution it does not provide any information if the one of the elements between PERSON, PLACE and WEAPON are correct, so there isn't any deduction in the state machine_node to faster derive the right solution.
For this reason since the big amount of movements that the robot has to perform to get hints and since for each movement corresponds a certain time in which the simulation has to wait, the simulation can require a lot of time to derive the solution, even with a small amount of elements (as in this case just three people, three places and three weapons). 
In order to limit the dealy generated by searcing for queries in an ontolgy with a very big amount of hypothesis, all the hypotesys (except from the solution) are removed from the ontology as long as it is discovered that they are imcoplete, inconsistent or that does not represent the right solution. In this way, even if the big amount of generated hints, the time required to ask for a query reamins constant and that permits to avoid slowdowns in the case in which the simulation would generate a very big number of hints.

In the case in which a faster simualtion is preferred, on possible future implementation could be the capability to remember the solutions that are wrong in order to exclude them when the oracle generates a new hint and to faster derive the correct solution. Another possibility could be the capability understand if some of the elmenents of the solution are correct when the oracle checks it in order to ask to the oracle not to send them anymore.

## Authors and contacts
The author of the repository is Alessio Roda, I'm a student at the Robotics Engineering Univeristy of Genoa, if you need you can contact me at this email address: alessioroda98@gmail.com






