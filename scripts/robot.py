#!/usr/bin/env python

from read_config import read_config
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
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
		self.pose = get_pose(x,y,theta)

class RobotLocalizer():

	def __init__(self):
		rospy.init_node("robot_node", anonymous=True)
		random.seed(0)
		self.json_data = read_config()
		self.orig_grid = None
		self.setup_subs()
		while self.orig_grid == None:
			rospy.sleep(0.5)

		self.initParticles()
		self.MapInstance = Map(self.orig_grid)
		self.laser_array = []
		self.constructLikelihood()
		self.move_robot()
		self.out_file_pub.publish(Bool(data=True))
		rospy.sleep(1)
		rospy.signal_shutdown("All Done.")
		rospy.spin()
			
	def move_robot(self):
		move_list = self.json_data["move_list"]
		for move_idx, move in enumerate(move_list):
			self.result_update_pub.publish(Bool(data=True))

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
				for idx,particle in enumerate(self.particles_array):			
					particle.pose = get_pose(particle.x, particle.y,particle.theta)	
					if self.MapInstance.get_cell(particle.x, particle.y) == 1:
						particle.weight = 0	
					self.pose_array.poses[idx] = particle.pose
					
				self.re_weight()
				self.normalize_weights()
				self.re_sample()
				self.particlecloud_pub.publish(self.pose_array)

	def re_sample(self):
		new_particles = []
		idx = int(random.random() * self.num_particles)
		beta = 0.0
		max_weight = max(self.particle_array, key= lambda p:p.weight)
		for num in range(self.num_particles):
			beta += random.random() * max_weight * 2.0
			selected_particle = self.particle_array[idx]
			while beta > selected_particle.weight:
				beta -= selected_particle.weight
				idx = (idx + 1) % self.num_particles
			
			x_noise = random.gauss(0, self.json_data["resample_sigma_x"])
			y_noise = random.gauss(0, self.json_data["resample_sigma_y"])
			angle_noise = random.gauss(0, self.json_data["resample_sigma_angle"])
			
			selected_particle.x += x_noise
			selected_particle.y += y_noise
			selected_particle.theta += angle_noise	

			selected_particle.pose = get_pose(selected_particle.x, selected_particle.y,
				selected_particle.theta)	
			self.pose_array.poses[num] = selected_particle.pose
	
			new_particles.append(selected_particle)
		self.particle_array = new_particles
				

	def re_weight(self):
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
	
	def normalize_weights(self):
		total_weight = sum([particle.weight for particle in 		
			self.particle_array])
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
		self.out_file_pub = rospy.Publisher("/map_node/sim_complete", Bool,
		queue_size=10)


if __name__ == '__main__':
	try:
		RobotLocalizer()
	except rospy.ROSInterruptException:
		pass
	
