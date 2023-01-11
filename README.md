# Formation Movement
This is my reseach project for the course "Gameplay Programming" at Howest University of Applied Sciences.
The whole project is made using the Elite-Engine. The list of the files added by me can be found at the bottom of this readme.
## Description of the topic
In my research I'm figuring out how to make different formations using flocking and steering behaviours. 
My goal was to create shapes without having to offset the position by fixed values, I wanted more organic shapes.

![FormationSwitching](https://user-images.githubusercontent.com/114002276/211860516-a65eda0d-d20f-4483-99b8-906b2ac22f74.gif)

The second part of my project was finding a way to make the units traverse the world together. I did this by calculating only one common path for the formation.

![FormationSplittingFixv2](https://user-images.githubusercontent.com/114002276/211861652-6ae09756-0175-4813-bd74-539276038716.gif)

## Design/implementation
So, what are steering behaviours about? These are functions that calculate the lineair and angular velocities of an agent to reach a given target. 
The basic steerings that I am using are: 
1. Seek - linear velocity towards the target 
2. Face - angular velocity towards the the target 

Next to them there are also steerings that can be used for agents in a group or a flock, these steerings take dissrent aspects of te neighboring agents into account:
3. Separation - linear velocity away from other neighbors
4. Cohesion - linear velocity towards other neighbors
5. Velocity match - matching average velocity of the neighbors

We can use multiple steering 
iii. Result!

iv. Conclusion/Future work

