#!/usr/bin/env python  
import rospy

import numpy

import tf
import tf2_ros
import geometry_msgs.msg

def publish_transforms():
    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"        
    """object_transform.transform.translation.x=0
    object_transform.transform.translation.y=1
    object_transform.transform.translation.z=1"""
    obj_rot=tf.transformations.quaternion_from_euler(0.79,0.0,0.79)
    object_transform.transform.rotation.x=obj_rot[0]
    object_transform.transform.rotation.y=obj_rot[1]
    object_transform.transform.rotation.z=obj_rot[2]
    object_transform.transform.rotation.w=obj_rot[3]
    R1=tf.transformations.euler_matrix(0.79,0.0,0.79, axes='sxyz')
    t1=numpy.array([0,1,1,1])
    #print(t1.T.shape)
    new_trans=numpy.dot(R1 , t1.T)
    object_transform.transform.translation.x=new_trans[0]
    object_transform.transform.translation.y=new_trans[1]
    object_transform.transform.translation.z=new_trans[2]
    br.sendTransform(object_transform)

    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"
    rob_rot = tf.transformations.quaternion_about_axis(1.5, (0,0,1))
    robot_transform.transform.rotation.x = rob_rot[0]
    robot_transform.transform.rotation.y = rob_rot[1]
    robot_transform.transform.rotation.z = rob_rot[2]
    robot_transform.transform.rotation.w = rob_rot[3]
    '''R2=numpy.array([[0,-1,0],[1,0,0],[0,0,1]])
    #print(R2)
    #print(R2.shape)
    t2=numpy.array([0,-1,0])
    #print(t2.T)
    #print(t2.T.shape)
    new_trans2=numpy.dot(R2 , t2.T)
    robot_transform.transform.translation.x=new_trans2[0]
    robot_transform.transform.translation.y=new_trans2[1]
    robot_transform.transform.translation.z=new_trans2[2] '''
    euler1 = tf.transformations.euler_from_quaternion(rob_rot)
    R2=tf.transformations.euler_matrix(euler1[0],euler1[1],euler1[2], axes='sxyz')
    t2=numpy.array([0,-1,0,1])
    #print(t1.T.shape)
    new_trans2=numpy.dot(R2 , t2.T)
    robot_transform.transform.translation.x=new_trans2[0]
    robot_transform.transform.translation.y=new_trans2[1]
    robot_transform.transform.translation.z=new_trans2[2]
    br.sendTransform(robot_transform)
 
    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"
    camera_transform.transform.translation.x=0.0
    camera_transform.transform.translation.y=0.1
    camera_transform.transform.translation.z=0.1
    my_rot=tf.transformations.quaternion_from_euler(0,0,0)
    camera_transform.transform.rotation.x=my_rot[0]
    camera_transform.transform.rotation.y=my_rot[1]
    camera_transform.transform.rotation.z=my_rot[2]
    camera_transform.transform.rotation.w=my_rot[3]
    br.sendTransform(camera_transform)
    
    
    
    #dist_unit_v=trans
    #print(trans)

if __name__ == '__main__':
    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    listener = tf.TransformListener()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        try:
            (trans,rot) = listener.lookupTransform( '/object_frame','/camera_frame', rospy.Time.now())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        rospy.sleep(0.05)
