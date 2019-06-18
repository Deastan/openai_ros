#!/usr/bin/env python

# Task iiwa environment. Create the world and the action

import rospy
import numpy as np
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
        
        # For GYM
        # Action space
        # +x, -x, +y, -y, +z. z 
        self.n_actions = 6
        self.action_space = spaces.Discrete(self.n_actions)
        # self.action_space = spaces.multi_discrete([5, 2, 2])

        # Observations space
        low = np.array([-0.8, -0.8, 0.0, 0.0, 0.0, 0.0, 0.0])
        # quat of (6.28, 6.28, 6.28) => (0.99534, 0.05761, 0.05761, 0.05162)
        high = np.array([0.8, 0.8, 1.266, 1.0, 1.0, 1.0, 1.0])
        self.observation_space = spaces.Box(low, high, dtype=np.float32)


        # TARGET
        target_x = 0.5#0.0#0.5
        target_y = 0.5#0.5
        target_z = 0.5#0.5
        self.target_position = []
        self.target_position.append(target_x)
        self.target_position.append(target_y)
        self.target_position.append(target_z)
        # Last position
        self.last_pose = [] #this is a pose
        self.last_observation = []
        # Current position
        self.current_pose = []
        self.current_observation = []
        # Movement result: True movement is done, false means the robot did't move
        self.movement_result = False

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
        joints_array = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]
        rospy.logdebug("Set Init Joints ==> " + str(joints_array))
        result = self.set_joint_action(joints_array)
        rospy.logdebug("Set Init Joints Works!")
        # rospy.sleep(3.0)
        return result

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """

        self.cumulated_reward = 0.0
        self.last_pose = self.get_endEffector_Object()
        self.current_pose = self.get_endEffector_Object()

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

        # print(kuka_pose)
        self.movement_result = self.set_endEffector_acttionToPose(kuka_pose)
        # self.movement_result = self.set_endEffector_pose(kuka_pose)
        
        # if self.movement_result == False:
        #     print("*******************************************************")
        #     print("COULD NOT MOVE")
        #     print("*******************************************************")
        # else:
        #     print("*******************************************************")
        #     print("COULD MOVED")
        #     print("*******************************************************")

        

        rospy.logdebug("END Set Action ==>"+str(action))

    # An observation is a vector with 7 rows which containing 
    # the pose of the hand effector
    # (x, y, z, quat.x, quat.y, quat.z, quat.w)
    def _get_obs(self):
        """
        
        """
        # rospy.logdebug("Start Get Observation ==>")
        self.last_pose = self.current_pose

        last_observation = []     
        last_observation.append(self.current_pose.position.x)
        last_observation.append(self.current_pose.position.y)
        last_observation.append(self.current_pose.position.z)
        last_observation.append(self.current_pose.orientation.x)
        last_observation.append(self.current_pose.orientation.y)
        last_observation.append(self.current_pose.orientation.z)
        last_observation.append(self.current_pose.orientation.w)

        self.last_observation = last_observation
        # get_obs_pose = self.get_endEffector_Object()
        self.current_pose = self.get_endEffector_Object()
        observation = []
        
        observation.append(self.current_pose.position.x)
        observation.append(self.current_pose.position.y)
        observation.append(self.current_pose.position.z)
        observation.append(self.current_pose.orientation.x)
        observation.append(self.current_pose.orientation.y)
        observation.append(self.current_pose.orientation.z)
        observation.append(self.current_pose.orientation.w)

        # Update observation 
        self.current_observation = observation
        return observation

    # Check if the goal is reached or not
    def _is_done(self, observations):
        """
        
        """

        done = False
        vector_observ_pose = []
        vector_observ_pose.append(observations[0])
        vector_observ_pose.append(observations[1])
        vector_observ_pose.append(observations[2])

        # Check if the hand effector is close to the target in cm!
        if self.distance_between_vectors(vector_observ_pose, self.target_position) < 0.03:
            done = True

        return done

    def _compute_reward(self, observations, done):
        """
        
        """
        
        # The sign depend on its function.
        total_reward = 0
        
        # create and update from last position
        last_position = []
        last_position.append(self.last_pose.position.x)
        last_position.append(self.last_pose.position.y)
        last_position.append(self.last_pose.position.z)

        # create and update current position
        current_position = []
        current_position.append(self.current_pose.position.x)
        current_position.append(self.current_pose.position.y)
        current_position.append(self.current_pose.position.z)

        # create the distance btw the two last vector
        distance_before_move = self.distance_between_vectors(last_position, self.target_position)
        distance_after_move = self.distance_between_vectors(current_position, self.target_position)

        # Give the reward
        if self.out_workspace:
            total_reward -=20
        else:
            if done:
                total_reward += 100
            else:
                if(distance_after_move - distance_before_move > 0):
                    # print("right direction")
                    total_reward += 2.0
                else:
                    print("wrong direction")
                    # total_reward -= +1.0

        # Time punishment
        total_reward -= 1.0


        rospy.logdebug("###############")
        rospy.logdebug("total_reward=" + str(total_reward))
        rospy.logdebug("###############")

        return total_reward

    # Internal TaskEnv Methods To be far from GYM 

    # Calculates the distance btw two position vectors
    def distance_between_vectors(self, v1, v2):
        """
        """
        dist = np.linalg.norm(np.array(v1) - np.array(v2))
        return dist
    
    def calculate_reward(self):
        print("Albus")

    # def check_workplace():
    #     print("Check workplace")
