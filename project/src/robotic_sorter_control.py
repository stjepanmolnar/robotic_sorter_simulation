#!/usr/bin/env python3
import rospy
import moveit_commander
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import AttachedCollisionObject
from math import pi
from tf import transformations, TransformListener
from skimage import feature
from skimage.transform import hough_line, hough_line_peaks
import numpy as np
import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt
import sympy as sp
from cv_bridge import CvBridge
import random
import os

position_survay = [1.25, 0, 1.2]
red_position = [0, 1.5, 0.5]
blue_position = [0, -1.5, 0.5]
green_position = [-1.5, 0, 0.5]
cube_position = [1.5, 0, 0.5]

image = Image()

def close_gripper():
	p = JointTrajectory()
	point = JointTrajectoryPoint()
	#Add both finger joints of the robot.
	p.header.frame_id = "palm"
	p.joint_names.append("joint_8")
	p.joint_names.append("joint_9")
	point.positions.append(0.045)
	point.positions.append(0.045)
	point.effort.append(500)
	point.effort.append(500)
	point.time_from_start = rospy.rostime.Duration(0.2)
	p.points.append(point)
	return p

def open_gripper():
	p = JointTrajectory()
	point = JointTrajectoryPoint()
	#Add both finger joints of the robot.
	p.header.frame_id = "palm"
	p.joint_names.append("joint_8")
	p.joint_names.append("joint_9")
	point.positions.append(0)
	point.positions.append(0)
	point.effort.append(0)
	point.effort.append(0)
	point.time_from_start = rospy.rostime.Duration(0.2)
	p.points.append(point)
	return p

def pose_goal(position):
	pose_goal = Pose()
	pose_goal.position.x = position[0]
	pose_goal.position.y = position[1]
	pose_goal.position.z = position[2]
	q = transformations.quaternion_from_euler(3.14, 0, 0.0)
	pose_goal.orientation.x = q[0]
	pose_goal.orientation.y = q[1]
	pose_goal.orientation.z = q[2]
	pose_goal.orientation.w = q[3]
	return pose_goal

def imgproc(RGB, camera, camera_height):
	img_w = camera['img_size'][0]
	img_h = camera['img_size'][1]
	plt.imshow(RGB)
	E=feature.canny(RGB[:,:,0])	
	H, theta, d = hough_line(E)	

	_, angles, distances = hough_line_peaks(H, theta, d, min_distance=20, min_angle=20, threshold=30)
	temp_distances = []
	temp_angles = []
	for _, angle, dist in zip(_, angles, distances):
		if((abs(angle) > 0.1 and abs(angle) < 1.5) or abs(angle) > 1.6):
			continue
		temp_distances.append(dist)
		temp_angles.append(angle)
	angles = temp_angles
	distances = temp_distances
	for angle, dist in zip(angles, distances):
		cs = np.cos(angle)
		sn = np.sin(angle)
		if np.abs(cs) >= np.abs(sn):
			v = np.array([0, img_h-1])
			u = (dist - sn * v) / cs
		else:
			u = np.array([0, img_w-1])
			v = (dist - cs * u) / sn
		plt.plot(u, v, 'g')	
	
	if(len(distances) > 6):
		return None, None
	#####			
	points = []
	#Points extraction
	plt.show()
	
	for i in range(len(distances)):
		for j in range(len(angles)):
			if(abs(abs(angles[i]) - abs(angles[j])) < 0.1):
				continue
            
			x = sp.Symbol('x', real=True)
			y = sp.Symbol('y', real=True)

			solutions = sp.solve([np.cos(angles[i])*x + np.sin(angles[i])*y - distances[i], np.cos(angles[j])*x + np.sin(angles[j])*y - distances[j]], [x, y], dict=True)
			points.append([float(solutions[0][x]), float(solutions[0][y])])
	
	points = np.array(points)
	
	#Compute t:
	t=[0,0,0]

	u = float(np.mean(points[:,0]))
	v = float(np.mean(points[:,1]))
	x = (u-camera['principal_point'][0])*camera_height/camera['focal_length']
	y = (v-camera['principal_point'][1])*camera_height/camera['focal_length']
	t=[x,y,camera_height]
	######
	trans, rot = transform_lisener.lookupTransform('base_link', 'camera_rgb_frame', rospy.Time.now()-rospy.Duration(0.1))
	cube_position = [0, 0, 0]
	cube_position[0] = trans[0] - t[1]
	cube_position[1] = trans[1] - t[0]
	cube_position[2] = trans[2] - t[2] + 0.3



	drop_position = [0, 0, 0]
	color = RGB[int(v), int(u), :]
	if color[0] > color[1] and color[0] > color[2]:
		drop_position = red_position
	elif color[1] > color[0] and color[1] > color[2]:
		drop_position = green_position
	else:
		drop_position = blue_position

	return cube_position, drop_position


def img_callback(msg):
    global image
    image = msg

def build_scene(spawner):
	tables = [
		["tableCube", cube_position],
		["tableRed", red_position],
		["tableGreen", green_position],
		["tableBlue", blue_position],
	]

	script_dir = os.path.dirname(__file__)
	file_path = os.path.join(script_dir, "..", "models", "box.urdf")

	with open(file_path, "r") as f:
		table_urdf = f.read()

	for table in tables:
		model = SpawnModelRequest()
		model.model_name = table[0]
		model.model_xml = table_urdf
		pose = Pose()
		pose.position.x = table[1][0]
		pose.position.y = table[1][1]
		pose.position.z = table[1][2]
		model.initial_pose = pose
		model.reference_frame = "base_link"
		spawner(model)

cube_counter = 0
def spawn_random_cube(spawner):
	global cube_counter
	script_dir = os.path.dirname(__file__)
	
	file_path = os.path.join(script_dir, "..", "models", "small_box_red.urdf")
	with open(file_path, "r") as f:
		box_red_urdf = f.read()

	file_path = os.path.join(script_dir, "..", "models", "small_box_blue.urdf")
	with open(file_path, "r") as f:
		box_blue_urdf = f.read()

	file_path = os.path.join(script_dir, "..", "models", "small_box_blue.urdf")
	with open(file_path, "r") as f:
		box_green_urdf = f.read()

	urdfs = [box_red_urdf, box_blue_urdf, box_green_urdf]

	selector = random.randint(0,2)
	position_x_noise = -0.15+random.random()*0.3
	position_y_noise = -0.15+random.random()*0.3

	model = SpawnModelRequest()
	model.model_name = "cube" + str(cube_counter)
	model.model_xml = urdfs[selector]
	pose = Pose()
	pose.position.x = cube_position[0] + position_x_noise
	pose.position.y = cube_position[1] + position_y_noise
	pose.position.z = cube_position[2]
	model.initial_pose = pose
	model.reference_frame = "base_link"
	spawner(model)
	cube_counter += 1

def set_home(move_group_arm):
	move_group_arm.set_named_target("home")
	plan = move_group_arm.go(wait=True)
	move_group_arm.stop()
	move_group_arm.clear_pose_targets()

def set_survay(move_group_arm, effort_publisher):
	goal = pose_goal(position_survay)

	move_group_arm.set_pose_target(goal)
	plan = move_group_arm.go(wait=True)
	move_group_arm.stop()
	move_group_arm.clear_pose_targets()

	command = open_gripper()
	effort_publisher.publish(command)

def set_dropoff(move_group_arm, effort_publisher, drop_position):
	goal = pose_goal(drop_position)

	dropoff_noise = random.random()

	goal.position.x = -0.2+goal.position.x+dropoff_noise*0.4
	goal.position.y = -0.2+goal.position.y+dropoff_noise*0.4
	goal.position.z = goal.position.z+0.5
	move_group_arm.set_pose_target(goal)
	plan = move_group_arm.go(wait=True)
	move_group_arm.stop()
	move_group_arm.clear_pose_targets()

	command = open_gripper()
	effort_publisher.publish(command)

def grasp_cube(move_group_arm, effort_publisher, cube_position):
	goal = pose_goal(cube_position)

	move_group_arm.set_pose_target(goal)
	plan = move_group_arm.go(wait=True)
	move_group_arm.stop()
	move_group_arm.clear_pose_targets()

	goal.position.z = goal.position.z-0.1
	move_group_arm.set_pose_target(goal)
	plan = move_group_arm.go(wait=True)
	move_group_arm.stop()
	move_group_arm.clear_pose_targets()

	command = close_gripper()
	effort_publisher.publish(command)
	rospy.sleep(2)

	goal.position.z = goal.position.z+0.2
	move_group_arm.set_pose_target(goal)
	plan = move_group_arm.go(wait=True)
	move_group_arm.stop()
	move_group_arm.clear_pose_targets()


rospy.init_node("move_rob_manip")
rate = rospy.Rate(1)
image_subscriber = rospy.Subscriber("/camera/image_raw", Image, img_callback)
transform_lisener = TransformListener()
effort_publisher = rospy.Publisher("/gripper_controller/command", JointTrajectory)
spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
build_scene(spawner)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name_arm = "arm"
group_name_gripper = "gripper"
move_group_arm = moveit_commander.MoveGroupCommander(group_name_arm)
move_group_gripper = moveit_commander.MoveGroupCommander(group_name_gripper)

move_group_arm.set_named_target("home")
plan = move_group_arm.go(wait=True)
move_group_arm.stop()
move_group_arm.clear_pose_targets()

for i in range(10):
	spawn_random_cube(spawner)
	
	set_home(move_group_arm)

	set_survay(move_group_arm, effort_publisher)

	temp_image = image
	bridge = CvBridge()
	image_cv = bridge.imgmsg_to_cv2(temp_image, desired_encoding='rgba8')
	box_width = 0.1
	box_height = 0.1
	camera_height = position_survay[2] - 0.4

	camera = {
		'img_size': [temp_image.width, temp_image.height],
		'focal_length': 503,
		'principal_point': [temp_image.width/2, temp_image.height/2],
	}


	cube_position, drop_position = imgproc(image_cv, camera, camera_height)
	tries = 0
	while cube_position is None:
		set_home(move_group_arm)
		rospy.sleep(1)
		set_survay(move_group_arm, effort_publisher)
		temp_image = image
		image_cv = bridge.imgmsg_to_cv2(temp_image, desired_encoding='rgba8')
		cube_position, drop_position = imgproc(image_cv, camera, camera_height)

	grasp_cube(move_group_arm, effort_publisher, cube_position)

	set_dropoff(move_group_arm, effort_publisher, drop_position)
	rospy.sleep(1)