#!/usr/bin/env python

import numpy
import rospy
import time

from openai_ros import robot_gazebo_env
# from openai_ros.openai_ros_common import ROSLauncher

# For the launch 
import roslaunch
import rospy
import rospkg
import os
import git
import sys

# msg
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class iiwaEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all Robot environments.
    """

    def __init__(self):
        """Initializes a new Robot environment.
        """
        # Variables that we give through the constructor. 

        rospy.logdebug("Entered Kuka iiwa Env")
        # print("************************************************")
        # print("Files: iiwa_env.py")
        # print("Class: iiwaEnv")
        # print("************************************************")

        # 1: iiwa 
        # 2: reflex
        choice = 1

        if choice == 1:
            rospackage_name="iiwa_gazebo"
            launch_file_name="put_iiwa_in_world.launch"
            pkg_string = "/home/roboticlab14/catkin_ws/src/iiwa_stack/iiwa_gazebo"
        
        elif choice == 2: 
            rospackage_name="reflex_gazebo"
            launch_file_name="reflex_test_training_gym.launch"
            pkg_string = "/home/roboticlab14/catkin_ws/src/reflex-ros-pkg/reflex_gazebo"
        

        # Prepare the path
        pkg_path = os.path.normpath(pkg_string)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False) #OK
        roslaunch.configure_logging(uuid)

        launch_dir = os.path.join(pkg_path, "launch") #ok
        path_launch_file_name = os.path.join(launch_dir, launch_file_name)
        # Launch the roslaunch
        launch = roslaunch.parent.ROSLaunchParent(uuid, [path_launch_file_name]) #ok
        launch.start()
        rospy.sleep(10)

        

        # Internal Vars
        self.controllers_list = [
            'PositionJointInterface_J1_controller',
            'PositionJointInterface_J2_controller',
            'PositionJointInterface_J3_controller',
            'PositionJointInterface_J4_controller',
            'PositionJointInterface_J5_controller',
            'PositionJointInterface_J6_controller',
            'PositionJointInterface_J7_controller',
            'joint_state_controller',
            'preshape_1_position_controller',
            'preshape_2_position_controller',
            'proximal_joint_1_position_controller',
            'proximal_joint_2_position_controller',
            'proximal_joint_3_position_controller',
            'distal_joint_1_position_controller',
            'distal_joint_2_position_controller',
            'distal_joint_3_position_controller',
            ]
        # print(self.controllers_list)

        self.robot_name_space = "iiwa"

        self.reset_controls_bool = True
        
        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        
        # super(iiwaEnv, self).__init__(controllers_list=self.controllers_list,
        #                                         robot_name_space=self.robot_name_space,
        #                                         reset_controls=reset_controls_bool)

        super(iiwaEnv, self).__init__(controllers_list=self.controllers_list,
                                             robot_name_space=self.robot_name_space,
                                             reset_controls=self.reset_controls_bool,
                                             start_init_physics_parameters=False,# instead if false
                                             reset_world_or_sim="WORLD")
        # super(iiwaEnv, self).__init__()
        
        print("********************* I AM HERE **************************")

        
        self.gazebo.unpauseSim()
        # Start Move Fetch Object, that checks all systems are ready
        #self.move_fetch_object = FetchSimpleMove()
        # Wait until Fetch goes to the init pose
        #self.move_fetch_object.init_position()

        # We pause until the next step
        #self.gazebo.pauseSim()


        # Publisher iiwa
        self.publishers_iiwa_array = []
        self._iiwa_joint_1_pub = rospy.Publisher(
            '/iiwa/PositionJointInterface_J1_controller/command', Float64, queue_size=1)
        self._iiwa_joint_2_pub = rospy.Publisher(
            '/iiwa/PositionJointInterface_J2_controller/command', Float64, queue_size=1)
        self._iiwa_joint_3_pub = rospy.Publisher(
            '/iiwa/PositionJointInterface_J3_controller/command', Float64, queue_size=1)
        self._iiwa_joint_4_pub = rospy.Publisher(
            '/iiwa/PositionJointInterface_J4_controller/command', Float64, queue_size=1)
        self._iiwa_joint_5_pub = rospy.Publisher(
            '/iiwa/PositionJointInterface_J5_controller/command', Float64, queue_size=1)
        self._iiwa_joint_6_pub = rospy.Publisher(
            '/iiwa/PositionJointInterface_J6_controller/command', Float64, queue_size=1)
        self._iiwa_joint_7_pub = rospy.Publisher(
            '/iiwa/PositionJointInterface_J7_controller/command', Float64, queue_size=1)
        self.publishers_iiwa_array.append(self._iiwa_joint_1_pub)
        self.publishers_iiwa_array.append(self._iiwa_joint_2_pub)
        self.publishers_iiwa_array.append(self._iiwa_joint_3_pub)
        self.publishers_iiwa_array.append(self._iiwa_joint_4_pub)
        self.publishers_iiwa_array.append(self._iiwa_joint_5_pub)
        self.publishers_iiwa_array.append(self._iiwa_joint_6_pub)
        self.publishers_iiwa_array.append(self._iiwa_joint_7_pub)

        # Suscriber iiwa
        # We use it to get the joints positions and calculate the reward associated to it
        rospy.Subscriber("/iiwa/joint_states", JointState,
                         self._joints_state_callback)


    # Methods needed by the RobotGazeboEnv
    # ----------------------------
    
    def _joints_state_callback(self, data):
        self.joint_states = data
    

    # def _check_all_systems_ready(self):
    #     """
    #     Checks that all the sensors, publishers and other simulation systems are
    #     operational.
    #     """
    #     # TODO
    #     print("I check all the sensors, but it is only a text without action")
    #     return True
    
    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()
    
    
    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()
        
    # Methods that the TrainingEnvironment will need.
    # ----------------------------

    # move the joint at the desired angle
    def move_iiwa_joints(self, joints_array):
        """
        It will move the iiwa joint at the values in joints_array 
        """
        i = 0
        for publisher_object in self.publishers_iiwa_array:
            joint_value = Float64()
            joint_value.data = joints_array[i]
            rospy.logdebug("JointsPos "+ str(i) + ": " + str(joint_value))
            publisher_object.publish(joint_value)
            i += 1
    
    # Get the joint state 
    def get_joint_states(self):
        return self.joint_states