# RoboND-Perception-Project

## Project: Perception Pick & Place
### Introduction
One of the primary challenges in warehousing goods is strking the optimal balance between storage density and labor. In an effort to reduce labor, automated storage and retrieval systems (ASRS) have historically been used with positive results especially in manufacturing where the shape and size of the inventory to be stored is highly consistent and repeatable. The handling of full pallet goods can similarly benifit owing to the standard dimensions of the pallets on which they are stored. However, when the dimensions of items to be stored are highly variable, and their unit of measure for storage can include loose items, the efficiency of traditional ASRS systems degrades due to several factors. Firstly, these systems are most efficient when they can move items in bulk since their cycle times are usually fixed regardless of the quantity of product moved. Secondly, many of these systems are not capable of storing items loosely without some form of container, bin or tray. These containers invariablly waste space since they are typically constrained to a few fixed sizes. One solution to this problem is to store multiple items in a single bin or shelving unit which can be handled reliably by the automation while recouping some of the lost efficiencies of utilizing bins. The one drawback of this approach being that the ASRS is capable of handling items at the bin level, not at the item level, and it is unlikely that an order will perfectly match all items stored within a bin. This has given rise to so-called 'goods-to-man' ASRS systems (i.e. Kiva, Autostore) that are designed to deliver the bins of mixed items to a human operator who then completes the task of removing the needed items from the bin before signaling the ASRS to return the bin to the storage area. These systems are semi-automated, since an operator has now been introduced to perform the final task of identifying, selecting and removing the desired product from the delivered bin. The logical evolution of a 'goods-to-man' solution would be a 'goods-to-robot' solution. Such a solution could potentially realize the labor savings of an ASRS system while maintaining the storage efficiency benifits of mixed bins.

One of the primary challenges to implementing a goods-to-robot system is that historically robots have struggled to match the speed and accuracy of human operators at the final picking task when leveraging mixed sku bins. However recent solutions from companies like Righthand Robotics and others are starting to realize near human speed at this task. 

In this exercise, the goal will be to use computer vision to perform the first two steps of the process required to autonomously pick and place mixed items. First the solution will take simulated input from an RGBD camera to recognize and identify the items presented in each of 3 scenarios. Following identification the solution will compute the locations of each objects centroid and then publish that information to the project for use in path planning and retrieval by the PR2 robot in the simulation. 

---
### Project Steps / Requirements
The successful project will complete the following steps and requiremnts:

1. Extract features and train an SVM model on new objects.
2. Write a ROS node and subscribe to the data being fed from the simulated RGBD camera
3. Since the simulated feed from teh RGBD camera includes noise, a filter should be implemented to remove it.
4. Utilize necessary filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels.
6. Calculate the centroid of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object.
8. Correctly identify objects in 3 scenarios with the following success rates:
  * Scenario 1: Identify 100% of the objects correctly (3 of 3 objects presented)
  * Scenario 2: Identify 80% of the objects correctly (4 of 5 objects presented)
  * Scneario 3: Identify 75% of the objects correctly (6 of 8 objects presented)

---
### Extracting and Training Features
Classifying the individual objects was dune using a Support Vector Machine that is trained on two feature vectors: 
  * A HSV color space histogram
  * A surface normal histogram

After trying several differnt combinations of SVC kernels and samples for each object, a satisfactorily good result was found with the following parameters:
  * Kernel = RBF
  * C = 4.0
  * n Samples = 30

More optimal configurations are possible, but these settings resulted in satisfactory results, as will be seen, in all 3 scenarios. The confusion matrix found with these settings are as follows:
![alt text](https://github.com/froohoo/RoboND-Perception-Project/blob/master/figure_1.png "Confusion Matrix Raw")
![alt text](https://github.com/froohoo/RoboND-Perception-Project/blob/master/figure_2.png "Confusion Matrix Normalized")

### Recieving, Filtering, and Conditoining the Point Cloud Data
#### Recieving the Point Cloud
To begin processing the camera data, the raw simulated feed was first subscribed to as follows:
```python
pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
```
#### Removing Noise
As previously mentioned, this point cloud data contains noise to mimick a real camera. To remove the noise a statistical filter was applied that compares each point it's neighbors and and then filters out points that are found to be statistical outliers in comparison. Two parameters are definable for the statistcal filter, the number of neighbors sampled and the number of standard deviations from the mean to be considered an outlier. Good results were found for this project with the following parameters:
  * Number of neighbors sampled = 6
  * Outlier std deviation from mean = .001
  
Results of applying the statistical filter with the above settings both before and after:
![alt text](https://github.com/froohoo/RoboND-Perception-Project/blob/master/no_stat_filter.png "Raw point could data")
![alt text](https://github.com/froohoo/RoboND-Perception-Project/blob/master/stat_filter.png "With statistical filter")

#### Conditioning Point Cloud to Improve Computational Performance
To improve performance, two steps were taken to reduce the complexity of the noise filtered pointcloud. First a voxel grid downsampling was performed to reduce the overall number of cloud points. This process averages the properties of pixels within user defined volumes to create a lower resolution, but more computationally manageable point clound. Leaf size chosen for this exersize was .005 as it provided adequate performance, while maintaining decent visual representations of the objects. 
The second step applied was to filter out all points that were not in the region of interest (i.e. outside of the table). This was done utilizing two passthrough filters: A filter in the z axis to remove all points below the table and a fiter applied to the y axis to remove all poits to the left and right of the table. One can see in the image below, that the points below the table, and to either side have been clipped in the image on the right, following passthrough filtering
![alt text](https://github.com/froohoo/RoboND-Perception-Project/blob/master/stat_filter.png "Before passthrough")
![alt text](https://github.com/froohoo/RoboND-Perception-Project/blob/master/passthrough.png "After passthrough")

### RANSAC segmentation and Euclidian Clustering
Once the noise is removed from the point cloud and it is at a resolution that is manageable for the virtual environment to process, the tasks of separating out and identifying the objects present can begin. Since the point cloud stil contains the table, the next step will be to identify all the points that belong to the table and remove those. What is left over should represent the objects, if any, that are sitting on the table. This was done using the Random Sample Consensus (RANSAC) method. Since the table represens a collection of planes, the RANSAC model utilized was SACMODEL_PLANE with a distance threshold of .01. Originally a value of .004 was tried, but it failed to capture the plane that represented the front edge of the table, so it was increased to .01.
Once the table was removed, the next step was to apply the Euclidian Clustering to segment the point clould in to smaller point clouds for each object. Also known as DBSCAN, as covered in lecture 18.7, the algorithm takes three parameters as input: the minumum/maximum number of points a cluster can contain and the maximum distance between cluster points. For the purposes of this project, the following settings provided adequate result:
  * Minimum Cluster Points = 75
  * Maximum Cluster Points = 2000
  * Maximum Point Distance = 0.02


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


