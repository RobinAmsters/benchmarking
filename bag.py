#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep  8 11:05:28 2017

@author: Robin Amsters
@email: robin.amsters@kuleuven.be
"""
import rosbag
import tf

import numpy as np

from transform import *

#==============================================================================
#               BAGFILE FUNCTIONS
#==============================================================================
def filter_experiment(pose, time, start_time, end_time):
    """
        Return only joint positions and times between a start and end time
    """

    for key in pose:
        joint = pose[key]
        joint_time = time[key]
        
        start_index = next(x[0] for x in enumerate(joint_time) if x[1] > start_time)
        end_index = next(x[0] for x in enumerate(joint_time) if x[1] > end_time)
        
        joint_x = joint[0][start_index:end_index]
        joint_y = joint[1][start_index:end_index]
        joint_z = joint[2][start_index:end_index]
        
        joint_time = joint_time[start_index:end_index]
        
        pose[key] = [joint_x, joint_y, joint_z]
        time[key] = joint_time
        
    return pose, time
        
#==============================================================================       

def get_bag_times(bagfile):
    """
        Get the start- and endtime of an experiment. These were manually 
        determined for each dataset based on the knee distance.
    """
#    start = {'25degrees1.bag': 15, 
#             '25degrees2.bag': 11, 
#             '25degrees3.bag': 11, 
#             '25degrees4.bag': 12, 
#             'fabre2.bag': 15, 
#             'fabre3.bag': 10,
#             'fabre4.bag': 15,
#             'fabre5.bag': 9,
#             'fabre6.bag': 15,
#             'mank1.bag': 0,
#             'mank2.bag': 0,
#             'mank3.bag': 0,
#             'mank4.bag': 0
#             }
#    end = {'25degrees1.bag': 27.5, 
#             '25degrees2.bag': 26, 
#             '25degrees3.bag': 27.5, 
#             '25degrees4.bag': 26, 
#             'fabre2.bag': 25, 
#             'fabre3.bag': 25,
#             'fabre4.bag': 25,
#             'fabre5.bag': 25,
#             'fabre6.bag': 28,
#             'mank1.bag': 0,
#             'mank2.bag': 0,
#             'mank3.bag': 0,
#             'mank4.bag': 0}
    start = {'25degrees1.bag': 15.0, 
             '25degrees2.bag': 15.0, 
             '25degrees3.bag': 17.5, 
             '25degrees4.bag': 15.0, 
             'fabre2.bag': 15.0, 
             'fabre3.bag': 15.0,
             'fabre4.bag': 15.0,
             'fabre5.bag': 15.0,
             'fabre6.bag': 15.0}
    
    end = {'25degrees1.bag': 25.0, 
             '25degrees2.bag': 25.0, 
             '25degrees3.bag': 27.5, 
             '25degrees4.bag': 25.0, 
             'fabre2.bag': 25.0, 
             'fabre3.bag': 25.0,
             'fabre4.bag': 25.0,
             'fabre5.bag': 25.0,
             'fabre6.bag': 25.0}
    prefix = '/media/storage/Box Sync/PhD/Papers/ICRA_2018/data_analysis/bag/'
    key = bagfile[len(prefix):len(bagfile)]
    
    start_time = start[key]
    end_time = end[key]
    
    return start_time, end_time
#==============================================================================
def get_bag_params(bagfile):
    """
        Get the peak range and distance for peak finder function. These were 
        manually determined for each dataset.
    """

    bag_heigth = {'25degrees1': 0.5, 
           '25degrees2': 0.5, 
           '25degrees3': 0.5, 
           '25degrees4': 0.5, 
           'fabre2': 0.35, 
           'fabre3': 0.35,
           'fabre4': 0.35,
           'fabre5': 0.35,
           'fabre6': 0.35,
             }
    
    bag_range = {'25degrees1': 20, 
           '25degrees2': 20, 
           '25degrees3': 20, 
           '25degrees4': 20, 
           'fabre2': 20, 
           'fabre3': 20,
           'fabre4': 20,
           'fabre5': 20,
           'fabre6': 20,
             }
    
    prefix = '/media/storage/Box Sync/PhD/Papers/ICRA_2018/data_analysis/bag/'
    key = bagfile[len(prefix):len(bagfile)]
    
    mph = bag_heigth[key]
    mpd = bag_range[key]
    
    
    return mph, mpd
    

#==============================================================================

def get_single_joint_data(bag, all_robot_pose, robot_time, joint_name, angle=0):
    """
        Function that filters bag files to obtain data from a single joint
        (e.g. head, neck, torso). This data is transformed according to the 
        camera angle. Data is transformed to world frame with robot pose.
    """
    x = np.array([])
    y = np.array([])
    z = np.array([])
    
    all_t = np.array([])
    first = 1
    index = 0
    
    # Add message values to collections
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        
        joint = msg.transforms[0].child_frame_id.split('_')
        translation = msg.transforms[0].transform.translation
        
        if joint[0] == joint_name: 
            
            # Get time difference from first timestamp
            if first:
                t_start = msg.transforms[0].header.stamp
                first = 0
        
        
            time = msg.transforms[0].header.stamp - t_start
            time_sec = time.to_sec()
            all_t = np.append(all_t, time_sec)
            
            # Get transformed position
            pose_cam = [translation.x , translation.y, translation.z]
            pose_cam_rot = cam_tilt_transform(pose_cam, angle)

#            index = len(x)
            t_robot = robot_time[index]
            while t_robot <= time_sec:
                index += 1
                t_robot = robot_time[index]
                
            robot_pose = np.array([all_robot_pose[0][index], all_robot_pose[1][index], all_robot_pose[2][index], all_robot_pose[3][index]])
            pose_world = robot_to_world(pose_cam_rot, robot_pose)

            x = np.append(x, pose_world[0,0])
            y = np.append(y, pose_world[1,0])
            z = np.append(z, pose_world[2,0])
    
    pose = [x,y,z]
    
    return pose, all_t

#==============================================================================

def get_dual_joint_data(bag, all_robot_pose, robot_time, joint_name_1, joint_name_2, angle=0):
    """
        Function that filters bag files to obtain data from a joint that is 
        specified with two names (e.g. right_elbow, left_foot, etc.).
        Data is transformed to world frame with robot pose. 
    """
    x = np.array([])
    y = np.array([])
    z = np.array([])
    
    all_t = np.array([])
    first = 1
    index = 0
    
    # Add message values to collections
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        
        joint = msg.transforms[0].child_frame_id.split('_')
        translation = msg.transforms[0].transform.translation
        
        if joint[0] == joint_name_1 and joint[1] == joint_name_2: 
            if first:
                t_start = msg.transforms[0].header.stamp
                first = 0
        
            time = msg.transforms[0].header.stamp - t_start
            time_sec = time.to_sec()
            all_t = np.append(all_t, time_sec)
            
            # Get transformed position
            pose_cam = [translation.x, translation.y, translation.z]
            pose_cam_rot = cam_tilt_transform(pose_cam, angle)
            
#            index = len(x)
            t_robot = robot_time[index]
            while t_robot <= time_sec:
                index += 1
                t_robot = robot_time[index]
                
            robot_pose = np.array([all_robot_pose[0][index], all_robot_pose[1][index], all_robot_pose[2][index], 
                                   all_robot_pose[3][index]])
            pose_world = robot_to_world(pose_cam_rot, robot_pose)
            
            x = np.append(x, pose_world[0,0])
            y = np.append(y, pose_world[1,0])
            z = np.append(z, pose_world[2,0])
            
    pose = [x,y,z]
    
    return pose, all_t

#==============================================================================

def get_robot_pose(bag):
    """
        Function that gets robot pose (position, orientation) from bag files.
        Robot frame should be named 'base_footprint'.
        Initial position is remapped to the origin
    """
    
    
    x = np.array([])
    y = np.array([])
    z = np.array([])
    th = np.array([])
    
    robot_t = np.array([])
    first = 1
    
    # Add message values to collections
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        
        joint = msg.transforms[0].child_frame_id.split('_')
        
        if joint[0] == 'base' and joint[1] == 'footprint': 
            
            # Get time difference from first timestamp
            if first:
                t_start = msg.transforms[0].header.stamp
                translation = msg.transforms[0].transform.translation
                rotation = msg.transforms[0].transform.rotation
                euler = tf.transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
                
                x_0 = translation.x
                y_0 = translation.y
                z_0 = translation.z
#                th_0 = euler[2]
                th_0 = 0
                
                first = 0
            
            else:
                
                translation = msg.transforms[0].transform.translation
                rotation = msg.transforms[0].transform.rotation
                euler = tf.transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
            
                t = msg.transforms[0].header.stamp - t_start
                t = t.to_sec()
                robot_t = np.append(robot_t, t)
                
                x = np.append(x, translation.x - x_0)
                y = np.append(y, translation.y - y_0)
                z = np.append(z, translation.z - z_0)
                th = np.append(th, euler[2] - th_0)
    
    robot_pose = [x, y, z, th]
    
    return robot_pose, robot_t

#==============================================================================    

def get_data(filePath, angle):
    """
        Function to obtain joint data contained in bag files. Joint data is 
        transformed to world frame based on robot pose.
    """
    # Open bag file
    bag = rosbag.Bag(filePath)
    
    # Get position data
    pose_robot, time_robot = get_robot_pose(bag)

    pose_torso, t_torso = get_single_joint_data(bag, pose_robot, time_robot, 'torso', angle=angle)
    pose_head, t_head = get_single_joint_data(bag, pose_robot, time_robot, 'head', angle=angle)
    pose_neck, t_neck = get_single_joint_data(bag, pose_robot, time_robot, 'neck', angle=angle)
    
    pose_l_shoulder, t_l_shoulder = get_dual_joint_data(bag, pose_robot, time_robot, 'left', 'shoulder', angle=angle)
    pose_l_elbow, t_l_elbow = get_dual_joint_data(bag, pose_robot, time_robot, 'left', 'elbow', angle=angle)
    pose_l_hand, t_l_hand = get_dual_joint_data(bag, pose_robot, time_robot, 'left', 'hand', angle=angle)
    pose_l_hip, t_l_hip = get_dual_joint_data(bag, pose_robot, time_robot, 'left', 'hip', angle=angle)
    pose_l_knee, t_l_knee = get_dual_joint_data(bag, pose_robot, time_robot, 'left', 'knee', angle=angle)
    pose_l_foot, t_l_foot = get_dual_joint_data(bag, pose_robot, time_robot, 'left', 'foot', angle=angle)
    
    pose_r_shoulder, t_r_shoulder = get_dual_joint_data(bag, pose_robot, time_robot, 'right', 'shoulder', angle=angle)
    pose_r_elbow, t_r_elbow = get_dual_joint_data(bag, pose_robot, time_robot, 'right', 'elbow', angle=angle)
    pose_r_hand, t_r_hand = get_dual_joint_data(bag, pose_robot, time_robot, 'right', 'hand', angle=angle)
    pose_r_hip, t_r_hip = get_dual_joint_data(bag, pose_robot, time_robot, 'right', 'hip', angle=angle)
    pose_r_knee, t_r_knee = get_dual_joint_data(bag, pose_robot, time_robot, 'right', 'knee', angle=angle)
    pose_r_foot, t_r_foot = get_dual_joint_data(bag, pose_robot, time_robot, 'right', 'foot', angle=angle)

#    # Get joint speeds
#    speed_robot = get_joint_speeds(pose_robot, t_robot)
#    
#    speed_torso = get_joint_speeds(pose_torso, t_torso)
#    speed_head = get_joint_speeds(pose_head, t_head)
#    speed_neck = get_joint_speeds(pose_neck, t_neck)
#    
#    speed_l_shoulder = get_joint_speeds(pose_l_shoulder, t_l_shoulder)
#    speed_l_elbow = get_joint_speeds(pose_l_elbow, t_l_elbow)
#    speed_l_hand = get_joint_speeds(pose_l_hand, t_l_hand)
#    speed_l_hip = get_joint_speeds(pose_l_hip, t_l_hip)
#    speed_l_knee = get_joint_speeds(pose_l_knee, t_l_knee)
#    speed_l_foot = get_joint_speeds(pose_l_foot, t_l_foot)
#    
#    speed_r_shoulder = get_joint_speeds(pose_r_shoulder, t_r_shoulder)
#    speed_r_elbow = get_joint_speeds(pose_r_elbow, t_r_elbow)
#    speed_r_hand = get_joint_speeds(pose_r_hand, t_r_hand)
#    speed_r_hip = get_joint_speeds(pose_r_hip, t_r_hip)
#    speed_r_knee = get_joint_speeds(pose_r_knee, t_r_knee)
#    speed_r_foot = get_joint_speeds(pose_r_foot, t_r_foot)    

    # Close bag file
    bag.close()
    
    pose = {'torso': pose_torso, 'head': pose_head, 'neck': pose_neck, 
            'l_shoulder': pose_l_shoulder, 'l_elbow': pose_l_elbow, 'l_hand': pose_l_hand, 
            'l_hip': pose_l_hip, 'l_knee': pose_l_knee, 'l_foot': pose_l_foot, 
            'r_shoulder': pose_r_shoulder, 'r_elbow': pose_r_elbow, 'r_hand': pose_r_hand, 
            'r_hip': pose_r_hip, 'r_knee': pose_r_knee, 'r_foot': pose_r_foot, 'robot': pose_robot 
            }
    
#    speed = {'torso': speed_torso, 'head': speed_head, 'neck': speed_neck, 
#            'l_shoulder': speed_l_shoulder, 'l_elbow': speed_l_elbow, 'l_hand': speed_l_hand, 
#            'l_hip': speed_l_hip, 'l_knee': speed_l_knee, 'l_foot': speed_l_foot, 
#            'r_shoulder': speed_r_shoulder, 'r_elbow': speed_r_elbow, 'r_hand': speed_r_hand, 
#            'r_hip': speed_r_hip, 'r_knee': speed_r_knee, 'r_foot': speed_r_foot, 'robot': speed_robot 
#            }
    
    t = {'torso': t_torso, 'head': t_head, 'neck': t_neck, 
            'l_shoulder': t_l_shoulder, 'l_elbow': t_l_elbow, 'l_hand': t_l_hand, 
            'l_hip': t_l_hip, 'l_knee': t_l_knee, 'l_foot': t_l_foot, 
            'r_shoulder': t_r_shoulder, 'r_elbow': t_r_elbow, 'r_hand': t_r_hand, 
            'r_hip': t_r_hip, 'r_knee': t_r_knee, 'r_foot': t_r_foot, 'robot': time_robot 
            }
    
    return pose, t