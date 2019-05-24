#!/usr/bin/env python

# iiwa environment which spawn the robot in gazebo world

import os
import git
import copy
import sys
import time

from openai_ros import robot_gazebo_env
# from openai_ros.openai_ros_common import ROSLauncher

# For the launch 
import roslaunch
import rospy
import rospkg

#moveit
import moveit_commander
from moveit_commander.conversions import pose_to_list

# msg
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import moveit_msgs.msg
from moveit_msgs.msg import MoveItErrorCodes


# Maths
import numpy
from tf.transformations import *
from math import pi

# Maths functions
def q_mult(q1, q2):
        result = q1
        w1 = q1.w
        x1 = q1.x
        y1 = q1.y
        z1 = q1.z

        w2 = q2.w
        x2 = q2.x
        y2 = q2.y
        z2 = q2.z
        # w2, x2, y2, z2 = q2
        result.w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        result.x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        result.y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        result.z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        
        
        return result

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
            'PositionJointInterface_trajectory_controller',
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

        self.gazebo.unpauseSim()
        # Start Move Fetch Object, that checks all systems are ready
        #self.move_fetch_object = FetchSimpleMove()
        # Wait until Fetch goes to the init pose
        #self.move_fetch_object.init_position()

        # We pause until the next step
        #self.gazebo.pauseSim()


        # Publisher iiwa
        # self.publishers_iiwa_array = []
        # self._iiwa_joint_1_pub = rospy.Publisher(
        #     '/iiwa/PositionJointInterface_J1_controller/command', Float64, queue_size=1)
        # self._iiwa_joint_2_pub = rospy.Publisher(
        #     '/iiwa/PositionJointInterface_J2_controller/command', Float64, queue_size=1)
        # self._iiwa_joint_3_pub = rospy.Publisher(
        #     '/iiwa/PositionJointInterface_J3_controller/command', Float64, queue_size=1)
        # self._iiwa_joint_4_pub = rospy.Publisher(
        #     '/iiwa/PositionJointInterface_J4_controller/command', Float64, queue_size=1)
        # self._iiwa_joint_5_pub = rospy.Publisher(
        #     '/iiwa/PositionJointInterface_J5_controller/command', Float64, queue_size=1)
        # self._iiwa_joint_6_pub = rospy.Publisher(
        #     '/iiwa/PositionJointInterface_J6_controller/command', Float64, queue_size=1)
        # self._iiwa_joint_7_pub = rospy.Publisher(
        #     '/iiwa/PositionJointInterface_J7_controller/command', Float64, queue_size=1)
        # self.publishers_iiwa_array.append(self._iiwa_joint_1_pub)
        # self.publishers_iiwa_array.append(self._iiwa_joint_2_pub)
        # self.publishers_iiwa_array.append(self._iiwa_joint_3_pub)
        # self.publishers_iiwa_array.append(self._iiwa_joint_4_pub)
        # self.publishers_iiwa_array.append(self._iiwa_joint_5_pub)
        # self.publishers_iiwa_array.append(self._iiwa_joint_6_pub)
        # self.publishers_iiwa_array.append(self._iiwa_joint_7_pub)

        # Suscriber iiwa
        # We use it to get the joints positions and calculate the reward associated to it
        # rospy.Subscriber("/iiwa/joint_states", JointState,
        #                  self._joints_state_callback)

        # rospy.sleep(20)
        # Create instance for moveIt
        self.moveit_object = MoveIiwa()
        
       
    # Methods needed by the RobotGazeboEnv
    # ----------------------------
    
    # def _joints_state_callback(self, data):
    #     self.joint_states = data

    # Set the joints
    def set_joint_action(self, joints_angle):
        result = self.moveit_object.set_joints_execute(joints_angle)

        return result

    # Get the position of the end effector
    def get_endEffector_Object(self):
        # print("get end effector pose")
        self.gazebo.unpauseSim()
        intermed_pose = self.moveit_object.get_endEffector_pose().pose
        self.gazebo.pauseSim()

        return intermed_pose
    
    

    # Vector action is the delta position (m) and angle (rad)
    # (x, y, z, R, P, Y)
    def set_endEffector_acttionToPose(self, action):
        
        # Current pose of the hand effector
        current_pose = geometry_msgs.msg.Pose()
        current_pose = self.moveit_object.get_endEffector_pose().pose
        # print("***************************************************")
        # print("current orientation")
        # print(current_pose.orientation)
        q_interm = quaternion_from_euler(action[3], action[4], action[5])
        q_interm[0] = current_pose.orientation.x
        q_interm[1] = current_pose.orientation.y
        q_interm[2] = current_pose.orientation.z
        q_interm[3] = current_pose.orientation.w
        # print("current q_interm")
        # print(q_interm)

        # print("***************************************************")
        # euler to quaternion 
        # R, P, Y = action[3], action[4], action[5]
        q_rot = quaternion_from_euler(action[3], action[4], action[5])
        # print("***************************************************")
        # print("desired rotation")
        # print(q_rot)
        # print("***************************************************")

        # create pose msg
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = current_pose.position.x + action[0]
        pose_goal.position.y = current_pose.position.y + action[1]
        pose_goal.position.z = current_pose.position.z + action[2]
        q_interm = quaternion_multiply(q_rot, q_interm)
        pose_goal.orientation.x = q_interm[0]
        pose_goal.orientation.y = q_interm[1]
        pose_goal.orientation.z = q_interm[2]
        pose_goal.orientation.w = q_interm[3]
        # pose_goal.orientation =  q_mult(q_rot, current_pose.orientation)

        result = self.moveit_object.set_endEffector_pose(pose_goal)

        return result
    
    # TODO CREATE THIS FUNCTION
    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        # TODO
        print("I check all the sensors, but it is only a text without action")
        return True
    
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
    # def move_iiwa_joints(self, joints_array):
    #     """
    #     It will move the iiwa joint at the values in joints_array 
    #     """
    #     i = 0
    #     for publisher_object in self.publishers_iiwa_array:
    #         joint_value = Float64()
    #         joint_value.data = joints_array[i]
    #         rospy.logdebug("JointsPos "+ str(i) + ": " + str(joint_value))
    #         publisher_object.publish(joint_value)
    #         i += 1
    
class MoveIiwa(object):
    def __init__(self):
        rospy.logdebug("MoveIiwaMoveIt Class init...")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.logdebug("moveit_commander initialised...")

        rospy.logdebug("Starting Robot Commander...")
        self.robot = moveit_commander.RobotCommander()
        rospy.logdebug("Starting Robot Commander...DONE")

        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.logdebug("PlanningSceneInterface initialised...DONE")
        
        self.group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        rospy.logdebug("MoveGroupCommander for arm initialised...DONE")

        # #GET INFO
        # planning_frame = self.group.get_planning_frame()
        # print "============ Reference frame: %s" % planning_frame

        # # We can also print the name of the end-effector link for this group:
        # eef_link = self.group.get_end_effector_link()
        # print "============ End effector: %s" % eef_link

        # # We can get a list of all the groups in the robot:
        # group_names = self.robot.get_group_names()
        # print "============ Robot Groups:", self.robot.get_group_names()

        # # Sometimes for debugging it is useful to print the entire state of the
        # # robot:
        # print "============ Printing robot state"
        # print self.robot.get_current_state()
        # print ""

    def set_joints_execute(self, joints_angle):
        
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = self.group.get_current_joint_values()
        for i in range(0,6):
            joint_goal[i] = joints_angle[i]

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        result = self.group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

        return result

    def set_endEffector_pose(self, pose):
        self.group.set_pose_target(pose)
        result = self.execute_trajectory()

        return result

    # Plan and Execute the trajectory planed for a given cartesian pose
    def execute_trajectory(self):
        self.plan = self.group.plan()
        result = self.group.go(wait=True)

        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()


        return result
    
    def get_endEffector_pose(self):
        # print("inside method: get_endEffector_pose => 1")
        endEffector_pose = self.group.get_current_pose()
        # print(endEffector_pose)
        # print("*********************************************************3")
        # print(self.robot.get_current_state())
        # print("inside method: get_endEffector_pose => 2")
        rospy.logdebug("EE POSE==>"+str(endEffector_pose))

        return endEffector_pose