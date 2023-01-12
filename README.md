# RTS Formation Movement
This is my reseach project for the course "Gameplay Programming" at Howest University of Applied Sciences.
The whole project written in c++ and is made using the Elite-Engine. The list of the files added by me can be found at the bottom of this readme.
## What is this project about?
In my research I'm figuring out how to make different formations using flocking and steering behaviours. 
My goal was to create shapes without having to offset the position by fixed values, I wanted a system for more organic shapes.

![FormationSwitching](https://user-images.githubusercontent.com/114002276/211860516-a65eda0d-d20f-4483-99b8-906b2ac22f74.gif)

The second part of my project was finding a way to make the units traverse the world together. I did this by calculating only one common path for the formation.

![FormationSplittingFixv2](https://user-images.githubusercontent.com/114002276/211861652-6ae09756-0175-4813-bd74-539276038716.gif)

## How is this implemented?
I started with a little setup: Every RTS-like game needs a way of selecting units and the location where the units should go. 
When you hold the left mouse button, you can move your mouse in any direction to create a selection rectangle. The units are assigned to the formation after releasing the button. I did this by checking if the middle position of any my agents is inside the currently selected rectangle.

The size and direction of the formation can also be adjusted by holding and dragging right mouse button at any location. If you just click at a location, the size and rotation will stay the same. The camera movement was already implemented in the engine, only thing I changed there was setting the middle mouse button for this purpose. 

![Selecting units](https://user-images.githubusercontent.com/114002276/211934440-12fe5a46-75eb-47a9-bd75-ee0cc3d4be3b.gif)

After that I started to work on the movement itself. Based on what I read online, calculating a path for every unit is not always the best solution. The main problems are that the units sometimes split in two separate groups and this solution also requires a lot more computing power.
I decided to calculate one common path, but every unit gets its own steering behaviour. 

### Steering behaviours

So, what are steering behaviours about? These are functions that calculate the linear and angular velocities of an agent based on the given target. 
The most common ones are "Seek" and "Face", they are respectivelly used for seeking a target and facing at it.

Other usefull steerings are so called "Flocking steerings", some examples are "Separation", "Cohesion" and "Velocity matching". This type of steering takes position and/or velocity of the neighboring agents into account.

The last type of steering behaviours that I use are "Combined steerings", with those behaviours you can easyly combine multiple steerings together. I only use the "blended steering" in this research. As it allows you to set how much influence every steering has on the end result.

### Organic formations

As I said, I wanted an organic formation, in other words without predictated positions. For this approach I decided to give my group only the target position where it should go, the right vector with magnitude equal to half of the size of the desired formation and formation type.

I started with implementing the line formation: my first idea was to "seek" the position on the line perpendicular to my agent. For this implementation I used the following math: dot product of perpendicular vector equals 0 and every position on a line can be described with: Begin point + (End point - Begin point) * t where 0 <= t <= 1.

I wanted to prevent clustering, so I blended my seek with separation and added face behaviour on top of that. This gave me a really promising result, but the problem was still there.

![LineClusteringError](https://user-images.githubusercontent.com/114002276/211951572-3da85803-5b16-4d74-9231-6aef58f48e41.gif)

I thought that adding a seek to the furthest point would help and it did, I quite liked the result, so I let it in the final program under "Loose movement" button.
After that I tried a similar technique with a circle. The math for that is: direction where agent comes from = current agent position - center of the circle. So, the desired seek position is: the center + normalized direction * radius of the circle. This ended up in an even larger clustering issue.

![ClusteringProblem](https://user-images.githubusercontent.com/114002276/211952501-823e8b1b-2bb5-48bc-91d5-725218d87515.gif)

Adding a seek to the opposite side helped a lot, but the code started to look to complicated. I came to the idea that I could use the current group center as the formation center and add another seek towards the end target. I combined both of the circle implementations for the "Loose movement".

There after I searched for a way to implement the "Seek desired location" in the line formation, I simply replaced the Begin and End positions by an offseted position from the current group center. The result was realy great.

### Multiple lines

Next step was implementing more than one line, the first method was to place the units in front of the center position in the first row and the others in the second row. This did not work out very wel, the main troubles were that the rows must contain the same ammount of agents, otherwise the whole group got pulled back. (the average position was pulled towards the larger group)

A second approach was to split the group in two separate groups, therefore I needed to create a manager, the "Formation" class. Once one of the settings in UI is changed, the agents are redistributed to the best fittng group. I achieve this by sorting all agents by a criterium, for example how much forward. The math that helps in sorting like this: dot product of the direction from an agent to the formation center with the forward vector of the formation, if this result is larger, it means the agent is in front, otherwise it is in the back.

### Back to path finding

We already implemented A* path finding with a small optimisation during the semester, but it was not 100% suitable for my needs. The problem was that the basic implementation doesn't account for the size of the group. My solution for that was to crop the portals by the size of the group. This worked in cases when a group was not to large. My second approach for the problem was to offset the final path point perpendicular to the direction where the agents will come from. As you can see below, this works nice for single line formations, but not for larger groups. Althought it isn't the perfect solution, I wil let this for now.

![Moving next to obstacle](https://user-images.githubusercontent.com/114002276/211942830-b2a34321-3877-4a3b-9fa8-f956dcfbf4ed.gif)

Notice that we need to use 90 degrees or -90 degrees from our direction based on the fact that the obstacle is at the left or right side. Figuring out how to deal with it took me a while. I found that when we calculate the next direction, we can compare it to the current direction and see if it turns left or right. For this purpose we can use the cross product. In our case a negative result means -90 degrees and a positive result 90 degrees. We can calculate the needed vector by using the next formula: 90 degrees rotated vector = {-vector.y,vector.x}. 

## What are the results?

## Next research topics
If you want, you can do a follow up project based on my work. The would recomend to work on the thing where I did not have time to finish:
1. Correct offset when moving next to obstacles. (The path finder self could also take the formation width into account)
2. Subgroups of the formation should always keep a relative position to eachother.
Other also interesting features would be:
1. Creating more basic shapes, for example by means of bezier-curves.
2. The formations could rotate to match the moving direction.

## Sources
https://www.gamedeveloper.com/programming/group-pathfinding-movement-in-rts-style-games
https://marclafr.github.io/Research-Group-Movement-RTS-/
https://github.com/EezehDev/AI-Formations
https://www.gamedeveloper.com/programming/coordinated-unit-movement

## List of added .h files
"projects/Movement/FormationMovement/UnitAgent.h"
"projects/Movement/FormationMovement/Group.h"
"projects/Movement/FormationMovement/Formation.h"
"projects/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
"projects/Movement/SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"
"projects/Movement/SteeringBehaviors/Flocking/FlockingSteeringBehaviors.h"
"projects/Movement/Pathfinding/NavMeshGraph/App_NavMeshGraph.h"
