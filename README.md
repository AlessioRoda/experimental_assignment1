# Cluedo simulation for the Experimental Robotics Lab course, Alessio Roda S4458313

The purpose of this repository is to generate a simulation for the Cluedo game, in which the robot moves from a place to another, asks hints to an oracle and tries to find a solution of the game. In order to do this, all the information about the scene and the hints perceived from the oracle are utilized to update an ARMOR ontology, with which the robot can reason to generate a solution for the game.

## Description of the architecture

![cluedo](https://user-images.githubusercontent.com/48511957/142238407-b648df07-2806-474c-a22e-d787d1638970.jpg)

The whole architecture wants to represent the steps of a cluedo game: there's a scene containing some places, people and weapons; once the game starts a random solution containing a person (the killer in the cluedo game), a place (place of the crime) and a weapon is generated. The robot starts in the center of the map, the oracle room, then it moves randomly for the scene to find some hints that can help him to find the solution; each time the robot enters in a new place a hint is provided from an oracle, then when the robot has collected some appropriated hints it tries to generate a solution by going in the oracle room and formally pronouncing a sentence that defines the solution, indicating the name of the killer, the place and the utilized weapon.


In order to run it just do it betttter  or even

```
rosrun armor execute it.emarolab.armor.ARMORMainService
```
then
```
roslaunch experimental_assignment1 sim.launch
```
