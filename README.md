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



In the Move step the robot has to move in a random place between the ones that are provided to collect hints (all the rooms can provide rooms, except to the Oracle_Room) and to perform the motion the state_machine node sends a 




In order to run it just do it betttter  or even

```
rosrun armor execute it.emarolab.armor.ARMORMainService
```
then
```
roslaunch experimental_assignment1 sim.launch
```
