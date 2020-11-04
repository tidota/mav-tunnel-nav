#!/usr/bin/env python

import argparse
import math
import os
import random
import subprocess
import sys

import rospkg
import yaml

def spawn(req):
	try:
		cmd = [
			'roslaunch',
			'mav_tunnel_nav',
			'spawn_robot.launch',
			'mav_name:=' + req.robot,
			'x:=' + str(req.x),
			'y:=' + str(req.y),
			'z:=' + str(req.z),
			'Y:=' + str(req.Y)
		]
		print('Running command: ' + ' '.join(cmd))
		p = subprocess.Popen(cmd)
		p.wait()
	except KeyboardInterrupt:
		pass
	finally:
		p.wait()

	return SpawnRobot(true)

if __name__ == '__main__':
	# Filter out any special ROS remapping arguments.
	# This is necessary if the script is being run from a ROS launch file.
	import rospy
	args = rospy.myargv(sys.argv)

	rospy.init_node('robot_sequence_spawner')

	s = rospy.Service('spawn_robot', SpawnRobot, spawn)

	rospy.spin()
