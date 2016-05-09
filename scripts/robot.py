#!/usr/bin/env python

from read_config import read_config
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray
from helper_functions import *
from sklearn.neighbors import KDTree
from map_utils import Map
import rospy, random, math

OCCUPIED = 100

class Particle():
	def __init__(self,x,y,theta, weight=0):
		self.x = x
		self.y = y
		self.theta = theta
		self.weight = weight

class RobotLocalizer():
	orig_grid = None

	def __init__(self):
		rospy.init_node("robot_node", anonymous=True)
		random.seed(0)
		self.json_data = read_config()
		setup_subs()
		initParticles()
		self.MapInstance = Map(orig_grid)
		self.laser_array = []
		constructLikelihood()
		move_robot()
		rospy.spin()
			
	def move_robot():
		move_list = self.json_data["move_list"]
		for move_idx, move in enumerate(move_list):
			theta = move[0]			
			move_function(theta,0)
			for particle in self.particles_array:
				particle.theta += theta

			for num_steps in move[2]:
				dist = move[1]			
				move_function(0, dist)
				for particle in self.particles_array:
					particle.x += math.cos(particle.theta) * dist
					particle.y += math.sin(particle.theta) * dist

				if move_idx == 0:
					x_noise = random.gauss(0, self.json_data
						["first_move_sigma_x"])
					y_noise = random.gauss(0, self.json_data
						["first_move_sigma_y"])
					theta_noise = random.gauss(0, self.json_data
						["first_move_sigma_angle"])
 					for particle in self.particles_array:
						particle.x += x_noise
						particle.y += y_noise
						particle.theta += theta_noise
				re_weight()
				normalize_weights()

	def re_weight():
		z_hit = self.json_data["laser_z_hit"]
		z_rand = self.json_data["laser_z_rand"]
		
		for particle in self.particle_array:
			pz_array = []
			for idx, val in enumerate(self.laser_data.ranges):	
				angle = particle.theta + (self.laser_data.angle_min +
					angle_increment * idx)
				x = particle.x + val * math.cos(angle)
				y = particle.y + val * math.sin(angle)
				likelihood_prob = self.MapInstance.get_cell(x,y)
				if not math.isnan(likelihood_prob):
					pz = z_hit * likelihodd_prob + z_rand
					pz_array.append(pz**3)
			p_total = sum(pz_array)
			particle.weight = (p_total+0.2) * particle.weight
	
	def normalize_weights():
		total_weight = sum([particle.weight for particle in self.particle_array])
		for particle in self.particle_array:
			particle.weight = particle.weight/total_weight

	def constructLikelihood(self):
		obstacles_array = []
		all_coord = []
		for r_idx, row in enumerate(self.MapInstance.grid):
			for c_idx, col in enumerate(row):
				all_coord.append((r_idx, c_idx))
				if self.MapInstance.grid[r_idx][c_idx] == OCCUPIED:
					obstacles_array.append((r_idx, c_idx))

		kdt = KDTree(obstacles_array, metric='euclidean')
		dist = kdt.query(all_coord, k=1)
		variance = self.json_data["laser_sigma_hit"]
		for idx, d in enumerate(dist):
			self.MapInstance.grid[idx] = get_gaussian(d, variance)

		self.likelihood_pub.publish(self.MapInstance.to_message())

	def get_gaussian(self,dist, variance):	
		power_e = math.e ** (-0.5 * ((float(dist)/variance)**2))
		return power_e

	def initParticles(self):
		pose_array = PoseArray()
		pose_array.header.stamp = rospy.Time.now()
		pose_array.header.frame_id = 'map'
		pose_array.poses = []

		self.particle_array = []
		num_particles = json_data["num_particles"]
		for particle in range(num_particles):
			x = random.uniform(0, orig_grid.info.width)
			y = random.uniform(0, orig_grid.info.height)
			theta = random.uniform(0, 2*math.pi)
			pose_obj = get_pose(x,y,theta)
			pose_array.poses.append(pose_obj)
			self.particle_array(Particle(x,y,theta, 1.0/num_particles))

		self.particlecloud_pub.publish(pose_array)

	def setup_subs(self):
		rospy.Subscriber("/map", OccupancyGrid, map_callback)
		rospy.Subscriber("/base_scan", LaserScan, laserscan_callback)

		self.particlecloud_pub = rospy.Publisher("/particlecloud", PoseArray,
			queue_size=10)
		self.likelihood_pub = rospy.Publisher("/likelihood_field", OccupancyGrid,
			latch=True, queue_size=10)

	def map_callback(self,data):
		self.orig_grid = data

	def laserscan_callback(self,data):
		self.laser_data = data

if __name__ == '__main__':
	try:
		RobotLocalizer()
	except rospy.ROSInterruptException:
		pass
	
