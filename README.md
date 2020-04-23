# Robot Sign Language 
A novel wireless communications method for autonomous vehicles using basic adaptive cruise control setup 
by encoding information in speed and perception with LiDAR. Implemented on [MuSHR software stack](mushr.io) from UW's Personal 
Robotics Lab.

![pic](wireless.png)

### Basic Idea Outline 
An adaptive cruise control setup where two cars are able to communicate to each other through slightly altering their speeds(encoding information in velocity). 

### Example Scenario
CarA travels at approximately 60mph constant speed and leads car2 at a fixed 20m distance. Car A encodes information in its speed by encoding a 0 as even speed(i.e 60 mph) and 1 as an odd speed (i.e 61 mph). Car B maintains the 20m distance using an adaptive cruise control setup. Now the speed of car B should be the same as car and and if we decode car B’s speed we can get the encoded message. 

### Challenge 
First we need to ensure the adaptive cruise control is good enough to stay at the desired distance and thus have the same speed as car A. This comes with the trade-off of efficiency. Thus we need to test the good balance between accuracy and efficiency of this method. 


## Setting up dependencies to run the simulation 
follow the instructions [here](https://mushr.io/tutorials/quickstart/) to download the required packages. 
Then clone this repo in the catkin_ws. 

