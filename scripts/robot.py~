#!/usr/bin/env python

from read_config import read_config
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseArray
from helper_functions import *
from sklearn.neighbors import KDTree
from map_utils import Map
from copy import deepcopy
import rospy, random, math
import numpy as np


class Particle():
	def __init__(self,x,y,theta, weight):
		self.x = x
		self.y = y
		self.theta = theta
		self.weight = weight
		self.pose = get_pose(x,y,theta)

class RobotLocalizer():

	def __init__(self):
		rospy.init_node("robot_node", anonymous=True)
		self.json_data = read_config()
		random.seed(self.json_data["seed"]) #
		self.orig_grid = None
		self.laser_data = None
		self.pose_array = None
		self.laser_array = []
		self.setup_subs()
		while self.orig_grid == None:
			rospy.sleep(0.5)

		self.initParticles()
		self.MapInstance = Map(self.orig_grid)
		self.constructLikelihood()
		while self.laser_data == None:
			rospy.sleep(0.5)

		self.move_robot()
		self.out_file_pub.publish(Bool(data=True))
		rospy.sleep(1)
		rospy.signal_shutdown("All Done.")
		
			
	def move_robot(self):
		move_list = self.json_data["move_list"]
		for move_idx, move in enumerate(move_list):
			self.result_update_pub.publish(Bool(data=True))

			#theta here is in degrees
			theta = move[0]			
			move_function(theta,0)
			for particle in self.particle_array:
				particle.theta += math.radians(theta)

			for num_steps in range(move[2]):
				print "step: ", num_steps
				dist = move[1]			
				move_function(0, dist)
				for particle in self.particle_array:
					particle.x += math.cos(particle.theta) * dist
					particle.y += math.sin(particle.theta) * dist

				if move_idx == 0:
					x_noise = random.gauss(0, self.json_data
						["first_move_sigma_x"])
					y_noise = random.gauss(0, self.json_data
						["first_move_sigma_y"])
					theta_noise = random.gauss(0, self.json_data
						["first_move_sigma_angle"])
 					for particle in self.particle_array:
						particle.x += x_noise
						particle.y += y_noise
						particle.theta += theta_noise	

				#for idx,particle in enumerate(self.particle_array):			
				#	particle.pose = get_pose(particle.x, particle.y,particle.theta)	
				#	#if math.isnan(self.MapInstance.get_cell(particle.x, particle.y)):
				#	#	particle.weight = 0	
				#	self.pose_array.poses[idx] = particle.pose
					
				self.re_weight()
				self.normalize_weights()
				self.re_sample()
				self.particlecloud_pub.publish(self.pose_array)

	def re_sample(self):
		weights = [particle.weight for particle in self.particle_array]
		samples = np.random.choice(self.particle_array,p=weights,size=self.num_particles)
		
		temp_list = []
		for idx, sample in enumerate(samples):
			particle = Particle(sample.x,sample.y,sample.theta,sample.weight)
			particle.pose = sample.pose
			x_noise = random.gauss(0, 1) #self.json_data["resample_sigma_x"]
			y_noise = random.gauss(0, 1) #self.json_data["resample_sigma_y"]
			angle_noise = random.gauss(0, self.json_data["resample_sigma_angle"])
			particle.x += x_noise
			particle.y += y_noise
			particle.theta += angle_noise
			temp_list.append(particle)

			particle.pose = get_pose(sample.x, sample.y, sample.theta)	
			self.pose_array.poses[idx] = particle.pose

		self.particle_array = deepcopy(temp_list)
				

	def re_weight(self):
		z_hit = self.json_data["laser_z_hit"]
		z_rand = self.json_data["laser_z_rand"]
		
		for particle in self.particle_array:
			val = self.MapInstance.get_cell(particle.x,particle.y)
			if math.isnan(val) or val == 1:
				particle.weight = 0
				continue
			pz_array = []
			for idx,val in enumerate(self.laser_data.ranges):	
				angle = particle.theta + (self.laser_data.angle_min +
					self.laser_data.angle_increment * idx)
				x = particle.x + val * math.cos(angle)
				y = particle.y + val * math.sin(angle)
				likelihood_prob = self.MapInstance.get_cell(x,y)
				
				if not math.isnan(likelihood_prob):
					pz = z_hit * likelihood_prob + z_rand
					pz_array.append(pz**3)
			p_total = sum(pz_array)
			particle.weight =  particle.weight * (1/(1 + math.exp(-p_total))) # *(p_total+0.2)
			 
	
	def normalize_weights(self):
		total_weight = sum([particle.weight for particle in 		
			self.particle_array])
		
		for particle in self.particle_array:
			particle.weight = particle.weight/total_weight

		total_weight = sum([particle.weight for particle in 		
			self.particle_array])
		
	def constructLikelihood(self):
		obstacles_array = []
		all_coord = []
		for r_idx, row in enumerate(self.MapInstance.grid):
			for c_idx, col in enumerate(row):
				x,y = self.MapInstance.cell_position(r_idx, c_idx)
				all_coord.append([x, y])
				if self.MapInstance.get_cell(x,y) != 0:
					obstacles_array.append([x, y])

		kdt = KDTree(obstacles_array)
		dist = kdt.query(all_coord, k=1)[0][:]
		variance = self.json_data["laser_sigma_hit"]
		probability = np.exp(-(dist**2)/ (2 *(variance**2)))
		self.MapInstance.grid = probability.reshape(self.MapInstance.grid.shape)

		self.likelihood_pub.publish(self.MapInstance.to_message())


	def initParticles(self):
		self.pose_array = PoseArray()
		self.pose_array.header.stamp = rospy.Time.now()
		self.pose_array.header.frame_id = 'map'
		self.pose_array.poses = []

		self.particle_array = []
		self.num_particles = self.json_data["num_particles"]
		for particle in range(self.num_particles):
			x = random.uniform(0, self.orig_grid.info.width)
			y = random.uniform(0, self.orig_grid.info.height)
			theta = random.uniform(0, 2*math.pi)
			pose_obj = get_pose(x,y,theta)
			self.pose_array.poses.append(pose_obj)
			self.particle_array.append((Particle(x,y,theta, 1.0/self.num_particles)))

		self.particlecloud_pub.publish(self.pose_array)

	def map_callback(self,data):
		self.orig_grid = data

	def laserscan_callback(self,data):
		self.laser_data = data

	def setup_subs(self):
		rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
		rospy.Subscriber("/base_scan", LaserScan, self.laserscan_callback)

		self.particlecloud_pub = rospy.Publisher("/particlecloud", PoseArray,
			latch=True, queue_size=10)
		self.likelihood_pub = rospy.Publisher("/likelihood_field", OccupancyGrid,
			latch=True, queue_size=10)
		self.result_update_pub = rospy.Publisher("/result_update", Bool, 
			queue_size=10)
		self.out_file_pub = rospy.Publisher("/sim_complete", Bool,
			queue_size=10)


if __name__ == '__main__':
	try:
		RobotLocalizer()
	except rospy.ROSInterruptException:
		pass
	
