#!/usr/bin/env python

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

        joints_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.move_iiwa_joints(joints_array)

        return True

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """

        # For Info Purposes
        self.cumulated_reward = 0.0

    def _set_action(self, action):
        """
        It sets the joints of monoped based on the action integer given
        based on the action number given.
        :param action: The action integer that sets what movement to do next.
        """

        rospy.logdebug("Start Set Action ==>"+str(action))

        # joints_array = [0.0, 0.55, 0.0, 0.77, 0.0, 0.0, 0.0]

        # self.move_iiwa_joints(action)
        

        rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        
        """
        rospy.logdebug("Start Get Observation ==>")

        observation = []

        return observation

    def _is_done(self, observations):
        """
        
        """

        # TODO
        done = True

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

    def is_in_desired_position(self, current_position, epsilon=0.05):
        """
        n
        """

        is_in_desired_pos = False

        return is_in_desired_pos

    def is_inside_workspace(self, current_position):
        """
        
        """
        is_inside = False

        return is_inside

