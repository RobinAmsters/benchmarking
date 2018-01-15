#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 19 10:43:37 2017

@author: Robin Amsters
@email: robin.amsters@kuleuven.be

ROS node to get verification data
"""

import rospy
import message_filters
import tf
import tf2_geometry_msgs
import tf2_ros

from geometry_msgs.msg import Pose2D
from EKF_VLP import pose2DWithCovarianceToPoseStamped, poseToPoseStamped, poseStampedToPose2DWithCovariance
from vlp.msg import Light, Pose2DWithCovariance, Intensities

def callback(measured_intensity, model_intensity, pose_est_with_cov, pose_ref_with_cov):
    
    data = Intensities()
    data.stamp = rospy.Time.now()
    
    data.z = measured_intensity.intensity
    data.measurement_header = measured_intensity.header
    
    # Transform to receiver frame  
    transform = tfBuffer.lookup_transform('base_link', 'receiver_link', rospy.Time())

    # Construct pose transform message    
    pose_for_transform = pose2DWithCovarianceToPoseStamped(pose_ref_with_cov)
    
    # Transform pose to receiver frame
    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_for_transform, transform)    
    
#    data.pose_ref = pose_ref
    data.pose_ref.x = pose_transformed.pose.position.x 
    data.pose_ref.y = pose_transformed.pose.position.y
    
    data.h = model_intensity.intensity
    data.model_header = model_intensity.header
    data.pose_est = pose_est_with_cov.pose
    data.cov_est = pose_est_with_cov.covariance
    
    data_pub.publish(data)


if __name__ == '__main__':
    
    #               INITIALIZE NODE
    slop = rospy.get_param('/sync_slop') # Slop for time synchroniser    
    rospy.init_node('collect_data', anonymous=True)
    
    # subscribe to relevant topics
    light_meas_sub = message_filters.Subscriber('/light_intensity', Light)
    light_model_sub = message_filters.Subscriber('/light_intensity_model', Light)
    pose_sub = message_filters.Subscriber('/pose', Pose2DWithCovariance)
#    pose_ref_sub = message_filters.Subscriber('/pose2D', Pose2D)
    pose_ref_sub = message_filters.Subscriber('/pose_DR', Pose2DWithCovariance)
    
    # Synchronize messages
#    ts = message_filters.ApproximateTimeSynchronizer([light_meas_sub, light_model_sub, pose_sub, pose_ref_sub], 10, slop)
    ts = message_filters.ApproximateTimeSynchronizer([light_meas_sub, light_model_sub, pose_sub, pose_ref_sub], 10, slop, allow_headerless=True)
    ts.registerCallback(callback)
    
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    # Initialize publisher           
    data_pub = rospy.Publisher('/verification_data', Intensities, queue_size=10)
    
    #               RUN NODE
    rospy.spin()