#!/usr/bin/env python

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

def load_gazebo_model(model_name, pose, model_type='sdf', frame="world"):
    # Load and Spawn SDF or URDF

    model_path = rospkg.RosPack().get_path('boris_tools')+"/models/"

    model_xml = ''

    with open (model_path + model_name + '.' + model_type, "r") as model_file:
        model_xml=model_file.read().replace('\n', '')

    if model_type=='sdf':
        srv_name = '/gazebo/spawn_sdf_model'  # Spawn SDF
    elif model_type=='urdf':
        srv_name = '/gazebo/spawn_urdf_model' # Spawn URDF
    else:
        print('model_spawner.py: Unknown type of model')

    rospy.wait_for_service(srv_name)
    try:
        spawn_srv = rospy.ServiceProxy(srv_name, SpawnModel)
        resp_srv = spawn_srv(model_name, model_xml, "/", pose, frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def delete_gazebo_models(model_name_list):
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        for model_name in model_name_list:
            resp_delete = delete_model(model_name)
    except rospy.ServiceException, e:
        print("Delete Model service call failed: {0}".format(e))

if __name__ == '__main__':
    print("Initializing node... ")
    rospy.init_node('gazebo_model_spawner')

    model_name_list = ['green_box', 'red_cylinder', 'green_cylinder', 'blue_box', 'red_box', 'blue_sphere']
    delete_gazebo_models(model_name_list)

    object_pose_zero = Pose(position=Point(x=0.5, y=0.6, z=0.725), orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
    object_pose_one = Pose(position=Point(x=0.5, y=0.4, z=0.725), orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
    object_pose_two = Pose(position=Point(x=0.5, y=0.2, z=0.725), orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
    object_pose_three = Pose(position=Point(x=0.5, y=0.0, z=0.725), orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
    object_pose_four = Pose(position=Point(x=0.5, y=-0.2, z=0.725), orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
    object_pose_five = Pose(position=Point(x=0.5, y=-0.4, z=0.725), orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
    object_poses = [object_pose_zero, object_pose_one, object_pose_two, object_pose_three, object_pose_four, object_pose_five]

    for i in range(len(model_name_list)): load_gazebo_model(model_name_list[i], object_poses[i], model_type='urdf')
