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

    model_name_list = ['object']
    delete_gazebo_models(model_name_list)

    object_pose = Pose(position=Point(x=0.45, y=0.6, z=0.725),
                      orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
    load_gazebo_model(model_name_list[0], object_pose, model_type='urdf')