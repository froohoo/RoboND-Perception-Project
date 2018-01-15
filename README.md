# RoboND-Perception-Project

## Project: Perception Pick & Place
### Introduction
One of the primary challenges in warehousing goods is strking the optimal balance between storage density and labor. In an effort to reduce labor, automated storage and retrieval systems (ASRS) have historically been used with positive results especially in manufacturing where the shape and size of the inventory to be stored is highly consistent and repeatable. The handling of full pallet goods can similarly benifit owing to the standard dimensions of the pallets on which they are stored. However, when the dimensions of items to be stored are highly variable, and their unit of measure for storage can include loose items, the efficiency of traditional ASRS systems degrades due to several factors. Firstly, these systems are most efficient when they can move items in bulk since their cycle times are usually fixed regardless of the quantity of product moved. Secondly, many of these systems are not capable of storing items loosely without some form of container, bin or tray. These containers invariablly waste space since they are typically constrained to a few fixed sizes. One solution to this problem is to store multiple items in a single bin or shelving unit which can be handled reliably by the automation while recouping some of the lost efficiencies of utilizing bins. The one drawback of this approach being that the ASRS is capable of handling items at the bin level, not at the item level, and it is unlikely that an order will perfectly match all items stored within a bin. This has given rise to so-called 'goods-to-man' ASRS systems (i.e. Kiva, Autostore) that are designed to deliver the bins of mixed items to a human operator who then completes the task of removing the needed items from the bin before signaling the ASRS to return the bin to the storage area. These systems are semi-automated, since an operator has now been introduced to perform the final task of identifying, selecting and removing the desired product from the delivered bin. The logical evolution of a 'goods-to-man' solution would be a 'goods-to-robot' solution. Such a solution could potentially realize the labor savings of an ASRS system while maintaining the storage efficiency benifits of mixed bins.

One of the primary challenges to implementing a goods-to-robot system is that historically robots have struggled to match the speed and accuracy of human operators at the final picking task when leveraging mixed sku bins. However recent solutions from companies like Righthand Robotics and others are starting to realize near human speed at this task. 

In this exercise, the goal will be to use computer vision to perform the first two steps of the process required to autonomously pick and place mixed items. First the solution will take simulated input from an RGBD camera to recognize and identify the items presented in each of 3 scenarios. Following identification the solution will compute the locations of each objects centroid and then publish that information to the project for use in path planning and retrieval by the PR2 robot in the simulation. 

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the â€œpick_place_routineâ€ rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  


