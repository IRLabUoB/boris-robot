#!/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Modified by: Ermano Arruda (exa371@bham.ac.uk)
# Modifications:
#   i) Removal control mode configs not pertinent to Boris (Kuka arms)
#  

"""
Boris Joint Trajectory Controller
    Unlike other robots running ROS, this is not a Motor Controller plugin,
    but a regular node using the SDK interface.
"""
import argparse
import importlib

import rospy
from dynamic_reconfigure.server import Server
from boris_tools.cfg import BorisPositionFFJointTrajectoryActionServerConfig as cfg

from boris_joint_trajectory_action.joint_trajectory_action import (
    JointTrajectoryActionServer,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

def start_server(limb, rate, mode, valid_limbs, sim = False):
    rospy.loginfo("Initializing node... ")
    rospy.init_node("sdk_{0}_joint_trajectory_action_server{1}".format(
                        mode, "" if limb == 'all_limbs' else "_" + limb,))

    rospy.loginfo("Initializing joint trajectory action server...")
    robot_name = "Boris"
    config_module = "boris_tools.cfg"
    # if mode == 'impedance':
    #     config_name = "BorisPositionFFJointTrajectoryActionServerServer"
    # else:
    #     config_name = "BorisPositionFFJointTrajectoryActionServerConfig"

    # import_name = '.'.join([config_module,config_name])

    # cfg = importlib.import_module(import_name)
    dyn_cfg_srv = Server(cfg, lambda config, level: config)
    jtas = []
    if limb == 'all_limbs':
        for current_limb in valid_limbs:
            jtas.append(JointTrajectoryActionServer(current_limb, dyn_cfg_srv,
                                                    rate, mode))
    else:
        jtas.append(JointTrajectoryActionServer(limb, dyn_cfg_srv, rate, mode, sim))


    def cleanup():
        for j in jtas:
            j.clean_shutdown()

    rospy.on_shutdown(cleanup)
    rospy.loginfo("Joint Trajectory Action Server Running. Ctrl-c to quit")
    rospy.spin()


def main():

    valid_limbs = ["left_arm","right_arm"]
    if not valid_limbs:
        rospy.logerr(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    # Add an option for starting a server for all valid limbs
    all_limbs = valid_limbs
    all_limbs.append("all_limbs")
    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=all_limbs,
        help="joint trajectory action server limb"
    )
    parser.add_argument(
        "-r", "--rate", dest="rate", default=100.0,
        type=float, help="trajectory control rate (Hz)"
    )
    parser.add_argument(
        "-m", "--mode", default='joint_impedance',
        choices=['joint_impedance'],
        help="control mode for trajectory execution"
    )
    parser.add_argument('--sim', dest='sim', action='store_true',
                         help="Specify whether this is a simulated robot")
    parser.set_defaults(sim=False)

    args = parser.parse_args(rospy.myargv()[1:])
    start_server(args.limb, args.rate, args.mode, valid_limbs, args.sim)


if __name__ == "__main__":
    main()