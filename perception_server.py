#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(ros_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    pcl_msg = ros_to_pcl(ros_msg)
    
    # TODO: Statistical Outlier Filtering
    # Create  the filter
    stat = pcl_msg.make_statistical_outlier_filter()
    # set the number of neighboring poitns to analyze for a given point
    stat.set_mean_k(6)
    #set threshold scale factor
    x = .001
    # Any point with a mean distance larger than global (mean distance+x*std_dev) will
    # be considered a statiscial outlier
    stat.set_std_dev_mul_thresh(x)
    # Call the filter function
    stat_filtered = stat.filter()

    
    # TODO: Voxel Grid Downsampling
    vox = stat_filtered.make_voxel_grid_filter()
    LEAF_SIZE = .005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    vox_filtered = vox.filter()

    # TODO: PassThrough Filter
    passthrough_z = vox_filtered.make_passthrough_filter()  
    # Assign axis and range to the passthrough filter object
    filter_axis = 'z'
    passthrough_z.set_filter_field_name(filter_axis)
    axis_min = .6
    axis_max = .8
    passthrough_z.set_filter_limits(axis_min, axis_max)
    # apply z filter (height)
    passthrough_filtered_z = passthrough_z.filter()
    # do same for y direction (left / right)
    passthrough_y = passthrough_filtered_z.make_passthrough_filter()
    filter_axis = 'y'
    passthrough_y.set_filter_field_name(filter_axis)
    axis_min = -.5
    axis_max =  .5
    passthrough_y.set_filter_limits(axis_min, axis_max)
    # apply y filter (left / right)
    passthrough_filtered = passthrough_y.filter()

    # TODO: RANSAC Plane Segmentation
    seg = passthrough_filtered.make_segmenter()

    # set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance
    # for segmenting the table

    max_distance = .01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier inidices and model coefficients
    # TODO: Extract inliers and outliers
    inliers, coefficients = seg.segment()
    # Extract table
    cloud_table = passthrough_filtered.extract(inliers, negative=False)
    # Extract objects
    cloud_objects = passthrough_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold as well as 
    # minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(75)
    ec.set_MaxClusterSize(2000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    print(len(cluster_indices))

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Assign a color corresponding to each segmented object in the scene
    cluster_color = get_color_list(len(cluster_indices))
    
    color_cluster_point_list = []
    
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                              rgb_to_float(cluster_color[j])])
    
    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)

    # TODO: Publish ROS messages
    pcl_statfil_pub.publish(pcl_to_ros(stat_filtered))
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)
    pcl_unfiltered_pub.publish(ros_msg)
    pcl_passthrough_pub.publish(pcl_to_ros(passthrough_filtered))

# Exercise-3 TODOs:

# Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        sample_cloud = pcl_to_ros(pcl_cluster)
        # Compute the associated feature vector
        chists = compute_color_histograms(sample_cloud, using_hsv=True)
        normals = get_normals(sample_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
        
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = sample_cloud
        detected_objects.append(do)
        
    rospy.loginfo('Detected {} objects {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    detected_objects_list = detected_objects
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()       #Valid values right, left
    pick_pose = Pose()
    place_pose = Pose()
    dict_list = []            #For our yaml, not our camel
        
    # Declare lists for labels and centroids from detected object list
    labels = []
    centroids = []
    
    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables (from our detected object list)
    for child in object_list:
        labels.append(child.label)
        # TODO: Get the PointCloud for a given object and obtain it's centroid
        points_arr = ros_to_pcl(child.cloud).to_array()
        centroid = np.mean(points_arr, axis=0)[:3]
        centroids.append(centroid)

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list and assign centroids to matching items
    for i, item in enumerate(object_list_param):
        name = item['name']
        try:
            idx = labels.index(name)
            item['centroid'] = centroids[idx].astype(type('float', (float,), {}))
            if item['group'] == 'green':
                arm = 'right'
            else:
                arm = 'left'
        except ValueError:
            print("Item %s not in detected object list" % name)
            continue
        place = np.array(((dbox for dbox in dropbox_param if dbox['name'] == arm).next())['position'])
        place = [float(i) for i in place]
        print(idx, item, place)
        
        # Assign the test_scene_num
        # Assign the object name
        # Assign the pick_pose
        # Assign 'place_pose' for the object
        # Assign the arm to be used for pick_place
        test_scene_num.data = 3
        object_name.data = name
        pick_pose.position.x = (item['centroid'][0])
        pick_pose.position.y = (item['centroid'][1])
        pick_pose.position.z = (item['centroid'][2])
        place_pose.position.x = place[0]
        place_pose.position.y = place[1]
        place_pose.position.z = place[2]
        arm_name.data = arm

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        dict_list.append(make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose))

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    send_to_yaml('perception_yaml.yml', dict_list)



if __name__ == '__main__':

 # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    
    # TODO: Create Publishers
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    pcl_statfil_pub = rospy.Publisher("/pcl_statfilered", PointCloud2, queue_size=1)
    pcl_unfiltered_pub = rospy.Publisher("/pcl_unfiltered", PointCloud2, queue_size=1)
    pcl_passthrough_pub = rospy.Publisher("/pcl_passthrough", PointCloud2, queue_size=1)
   
    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
