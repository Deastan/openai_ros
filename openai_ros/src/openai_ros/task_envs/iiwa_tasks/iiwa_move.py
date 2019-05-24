#!/usr/bin/env python

# Task iiwa environment. Create the world and the action

import rospy
import numpy
from gym import spaces
from openai_ros.robot_envs import iiwa_env
from gym.envs.registration import register

# from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
# from openai_ros.openai_ros_common import ROSLauncher
import os

# For the launch 
import roslaunch
import rospy
import rospkg
import git
import sys

class iiwaMoveEnv(iiwa_env.iiwaEnv):
    def __init__(self):
        
        rospy.logdebug("Start iiwaMoveEnv INIT...")

        # Set the package, launch file we need 
        ros_ws_abspath = "/home/roboticlab14/catkin_ws/"
        rospackage_name="iiwa_gazebo"
        launch_file_name="start_world.launch"

        # Prepare the path to run the launchfile
        pkg_string = "/home/roboticlab14/catkin_ws/src/iiwa_stack/iiwa_gazebo"
        pkg_path = os.path.normpath(pkg_string)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False) #OK
        roslaunch.configure_logging(uuid)
        launch_dir = os.path.join(pkg_path, "launch") #ok
        path_launch_file_name = os.path.join(launch_dir, launch_file_name)
        # Launch the roslaunch
        launch = roslaunch.parent.ROSLaunchParent(uuid, [path_launch_file_name]) #ok
        launch.start()
        # rospy.sleep(10)
        # launch.shutdown()


        # Variables of the class
        self.cumulated_reward = 0.0

        # TARGET
        target_x = 0.5
        target_y = 0.5
        target_z = 0.5
        target_position = []
        target_position.append(target_x)
        target_position.append(target_y)
        target_position.append(target_z)

        # Here we will add any init functions prior to starting the MyRobotEnv
        # super(iiwaMoveEnv, self).__init__(ros_ws_abspath)
        super(iiwaMoveEnv, self).__init__()

        

        rospy.logdebug("END iiwaMoveEnv INIT...")

    # Method inherited definition
    
    def _set_init_pose(self):
        """
        Sets the Robot in its init linear and angular speeds
        and lands the robot. Its preparing it to be reseted in the world.
        """
        rospy.logdebug("Start Set Initi Joints")
        joints_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rospy.logdebug("Set Init Joints ==> " + str(joints_array))
        result = self.set_joint_action(joints_array)
        rospy.logdebug("Set Init Joints Works!")
        return result

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """

        # For Info Purposes
        self.cumulated_reward = 0.0

    # The action here are the delta for the 6 first row
    # The delta position are in m
    # The angle are in Euler in rad 
    # action = [ x, y, z, R, P, Y ]
    def _set_action(self, action):
        """
        It sets the joints of monoped based on the action integer given
        based on the action number given.
        :param action: The action integer that sets what movement to do next.
        """

        rospy.logdebug("Start Set Action ==>" + str(action))

        # self.move_iiwa_joints(action)
        kuka_pose = []
        # for i in range(0,6):
        kuka_pose.append(action[0])
        kuka_pose.append(action[1])
        kuka_pose.append(action[2])
        kuka_pose.append(action[3])
        kuka_pose.append(action[4])
        kuka_pose.append(action[5])

        print(kuka_pose)
        self.set_endEffector_acttionToPose(kuka_pose)
        

        rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        
        """
        rospy.logdebug("Start Get Observation ==>")
        get_obs_pose = self.get_endEffector_Object()
        observation = []
        
        observation.append(get_obs_pose.position.x)
        observation.append(get_obs_pose.position.y)
        observation.append(get_obs_pose.position.z)
        observation.append(get_obs_pose.orientation.x)
        observation.append(get_obs_pose.orientation.y)
        observation.append(get_obs_pose.orientation.z)
        observation.append(get_obs_pose.orientation.w)


        return observation

    def _is_done(self, observations):
        """
        
        """

        # TODO
        done = False
        # if ()

        return done

    def _compute_reward(self, observations, done):
        """
        
        """

        # The sign depend on its function.
        total_reward = 0

        rospy.logdebug("###############")
        rospy.logdebug("total_reward=" + str(total_reward))
        rospy.logdebug("###############")

        return total_reward

    # Internal TaskEnv Methods

    def calculate_distance_between(self, v1, v2):
        """
        Calculated the Euclidian distance between two vectors given as python lists.
        """
        dist = np.linalg.norm(np.array(v1)-np.array(v2))
        return dist

