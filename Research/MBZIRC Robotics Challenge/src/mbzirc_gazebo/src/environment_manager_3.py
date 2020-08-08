#!/usr/bin/env python

import rospy
import sys
import random
import rospkg
import math
import tf
from subprocess import call
from random import shuffle
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

class Environment:
    TARGET_SIZE = 'target_size'
    TARGET_COLOR = 'target_color'
    TARGET_MOVEMENT = 'target_movement'
    MODEL_TARGET_PATH = '/models/target'
    MODEL_STAND_PATH = '/models/target_stand'
    TEMPLATE_FILE = 'model.sdf.erb'
    TARGET_BIN = '/models/target_bin/model.sdf'

    def __init__(self, service_proxy, package_directory, min_x, min_y, max_x, max_y):
        self.service_proxy = service_proxy
        self.package_directory = package_directory
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.target_id = 0

    def spawn_targets(self):
        red_target_model = open("{0}/{1}/model_red.sdf".format(self.package_directory, self.MODEL_TARGET_PATH), 'r').read()
        green_target_model = open("{0}/{1}/model_green.sdf".format(self.package_directory, self.MODEL_TARGET_PATH), 'r').read()
        blue_target_model = open("{0}/{1}/model_blue.sdf".format(self.package_directory, self.MODEL_TARGET_PATH), 'r').read()
        yellow_target_model = open("{0}/{1}/model_yellow.sdf".format(self.package_directory, self.MODEL_TARGET_PATH), 'r').read()
        orange_target_model = open("{0}/{1}/model_orange.sdf".format(self.package_directory, self.MODEL_TARGET_PATH), 'r').read()
        static_stand_model = open("{0}/{1}/model_static.sdf".format(self.package_directory, self.MODEL_STAND_PATH), 'r').read()
        moving_stand_model = open("{0}/{1}/model_moving.sdf".format(self.package_directory, self.MODEL_STAND_PATH), 'r').read()
        for i in range(0, 3):
            self.spawn_target(red_target_model, static_stand_model)
        for i in range(0, 4):
            self.spawn_target(green_target_model, static_stand_model)
        for i in range(0, 3):
            self.spawn_target(blue_target_model, static_stand_model)
        for i in range(0, 10):
            self.spawn_target(yellow_target_model, moving_stand_model)
        for i in range(0, 3):
            self.spawn_target(orange_target_model, static_stand_model)

    def spawn_target(self, target_model, stand_model):
        initial_pose = Pose()
        initial_pose.position.x = random.uniform(self.min_x, self.max_x)
        initial_pose.position.y = random.uniform(self.min_y, self.max_y)
        initial_pose.position.z = 0.053
        quaternion = tf.transformations.quaternion_from_euler(0, 0, random.uniform(0, 2 * math.pi))
        initial_pose.orientation.x = quaternion[0]
        initial_pose.orientation.y = quaternion[1]
        initial_pose.orientation.z = quaternion[2]
        initial_pose.orientation.w = quaternion[3]
        self.service_proxy('stand' + str(self.target_id), stand_model, 'targets', initial_pose, 'world')
        initial_pose.position.z += 0.20325
        self.service_proxy('target' + str(self.target_id), target_model, 'targets', initial_pose, 'world')
        self.target_id += 1

    def spawn_target_bin(self):
        initial_pose = Pose()
        initial_pose.position.y = -25.0
        initial_pose.position.z = 0.055
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        initial_pose.orientation.x = quaternion[0]
        initial_pose.orientation.y = quaternion[1]
        initial_pose.orientation.z = quaternion[2]
        initial_pose.orientation.w = quaternion[3]
        sdf_model = open(self.package_directory + self.TARGET_BIN, 'r').read()
        self.service_proxy('target_bin', sdf_model, 'targets', initial_pose, 'world')

    def generate_target_models(self):
        model_directory = self.package_directory + self.MODEL_TARGET_PATH
        self.generate_target(model_directory, 'small', 'red')
        self.generate_target(model_directory, 'small', 'green')
        self.generate_target(model_directory, 'small', 'blue')
        self.generate_target(model_directory, 'small', 'yellow')
        self.generate_target(model_directory, 'large', 'orange')

    def generate_stand_models(self):
        model_directory = self.package_directory + self.MODEL_STAND_PATH
        self.generate_stand(model_directory, 'static')
        self.generate_stand(model_directory, 'moving')

    def generate_target(self, model_directory, target_size, target_color):
        model_template = model_directory + '/' + self.TEMPLATE_FILE
        command = ' '.join([
            'erb',
            "{0}={1}".format(self.TARGET_SIZE, target_size),
            "{0}={1}".format(self.TARGET_COLOR, target_color),
            model_template, '>', "{0}/model_{1}.sdf".format(model_directory, target_color)
        ])
        print command
        call(command, shell=True)

    def generate_stand(self, model_directory, target_movement):
        model_template = model_directory + '/' + self.TEMPLATE_FILE
        command = ' '.join([
            'erb',
            "{0}={1}".format(self.TARGET_MOVEMENT, target_movement),
            model_template, '>', "{0}/model_{1}.sdf".format(model_directory, target_movement)
        ])
        print command
        call(command, shell=True)

if len(sys.argv) == 5:
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    rospy.init_node('environment_manager_3')
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    service_proxy = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    package_directory = rospkg.RosPack().get_path('mbzirc_gazebo')
    environment = Environment(service_proxy, package_directory, -x/2, -y/2, x/2, y/2)
    environment.generate_target_models()
    environment.generate_stand_models()
    environment.spawn_targets()
    environment.spawn_target_bin()
    rospy.spin()
