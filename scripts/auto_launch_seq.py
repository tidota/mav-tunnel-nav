#!/usr/bin/env python

import argparse
import math
import os
import random
import subprocess
import sys

from mav_tunnel_nav.srv import SpawnRobot,SpawnRobotResponse

import rospkg
import yaml

proc_list = []
dict_robot = {}
dict_comm = {}

def spawn(req):
	global proc_list
	global dict_robot
	global dict_comm
	try:
		cmd = [
			'roslaunch',
			'mav_tunnel_nav',
			'spawn_robot.launch',
			'mav_name:=' + req.robot,
			'x:=' + str(req.x),
			'y:=' + str(req.y),
			'z:=' + str(req.z),
			'Y:=' + str(req.Y),
			'map_only:=' + str(dict_robot[req.robot]['map_only']),
			'map_filename:=' + str(dict_robot[req.robot]['map_filename']),
			'output:=' + str(dict_robot[req.robot]['output']),
			'auto_pilot_type:=' + str(dict_robot['auto_pilot_type'])
		]
		if 'save_traj' in dict_robot[req.robot] and dict_robot[req.robot]['save_traj']:
			cmd += ['save_traj:=true']
		if req.auto_enable_by_slam:
			cmd += ['auto_enable_by_slam:=true']
		else:
			cmd += ['auto_enable_by_slam:=false']
		print('Running command: ' + ' '.join(cmd))
		proc_list += [subprocess.Popen(cmd)]
	except KeyboardInterrupt:
		pass
	finally:
		pass

	return SpawnRobotResponse(True)

if __name__ == '__main__':
	# Filter out any special ROS remapping arguments.
	# This is necessary if the script is being run from a ROS launch file.
	import rospy
	args = rospy.myargv(sys.argv)

	rospy.init_node('robot_sequence_spawner')

	rospack = rospkg.RosPack()
	try:
		f = open(rospack.get_path('mav_tunnel_nav') + '/config/robot_settings/comm.yaml', 'r')
		ver = [float(x) for x in yaml.__version__.split('.')]
		if ver[0] >= 5 and ver[1] >= 1:
			dict_comm = yaml.load(f.read(), Loader=yaml.FullLoader)
		else:
			dict_comm = yaml.load(f.read())
	except Exception as e:
		print(e)

	try:
		f = open(rospack.get_path('mav_tunnel_nav') + '/config/robot_settings/robots.yaml', 'r')
		ver = [float(x) for x in yaml.__version__.split('.')]
		if ver[0] >= 5 and ver[1] >= 1:
			dict_robot = yaml.load(f.read(), Loader=yaml.FullLoader)
		else:
			dict_robot = yaml.load(f.read())
	except Exception as e:
		print(e)

	s = rospy.Service(dict_comm['spawn_service_name'], SpawnRobot, spawn)

	rospy.spin()

	for p in proc_list:
		p.wait()
