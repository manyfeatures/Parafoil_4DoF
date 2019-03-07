import numpy as np
from scipy.integrate import odeint
from collections import UserDict # Can they be used to redefine some methods?
from IPython.core.debugger import set_trace
from parafoil_env import *
from parafoil_utils import *
from parafoil_params import *
from parafoil_control import *
			
class ParaDynamics(Parafoil):
	"""Trajectory calculation tools"""
	def __init__(self, time_steps=100): # Insert some parameters
		super().__init__() # ? 

		self.time_steps = time_steps
		
		self.update_period = 1 # Sec
		
		self.angles = {
			"yaw": 0,
			"roll": np.pi/12,
			"pitch": np.pi/2
		} # Parafoil orientation

		# Fix it
		self.init_params = [
							self.velocities["forward"],
							self.velocities["down"],
							self.angles["yaw"]
							] 
				
		# Needed?
		# Tuples?!? 
		self.trajectory = [tuple((self.pos["longitude"],
								 self.pos["latitude"],
								 self.pos["height"]))]  # trajectory history array 		
		self.time_fly = [Env.time] # time history array
		self.vel_array = [tuple((self.velocities["forward"],
								self.velocities["sideways"],
								self.velocities["down"]))] # velocities history array 
		self.angles_array = [tuple((self.angles["yaw"],
								   self.angles["roll"],
								   self.angles["pitch"]))] # velocities history array 
		
		
	def traj_init(self, init_params, t): 
		#self.update_init_params(init_params) 
		#self.update_pos_time_speed(dt) 
		
		#print(Env.time)
		#print("self.init_params: {0}, self.velocities {1}".format(self.init_params, self.velocities))
		
		return [
	    self.diff_forward_velocity(t), 
	    self.diff_down_velocity(t),
	    self.diff_yaw(t)]
		
	def traj_sol(self): 
		dt_list = np.linspace(0, self.update_period, self.time_steps)
		while self.pos["height"] > 0: # Max step 1 s
			print("Height: ", self.pos["height"])
			#Initialize roll conditions
			y0 = self.initialize_params()
			self.compute_all_features(y0, dt_list) # compute velocities, coordinates, points and log
			#set_trace()

	def initialize_params(self):
		init_params = [self.velocities["forward"],
		 			   self.velocities["down"],
		  			   self.angles["yaw"]]
		return init_params

	def compute_all_features(self, y0, dt_list):
		"""	
			- 2 (3) velocities 
			- 3 angles
			- 3 positions 
		"""
		# Calculation of features
		res_ = self.compute_and_optimize_num_diff_eqs(y0, dt_list) # forward, down speeds and yaw
		#set_trace()
		roll_ = self.roll_vector(dt_list) # use cycle()
		
		# Initialize roll 
		# self.angel["roll"]  = roll_[-1]
		# It is being done below

		longitude_, latitude_, height_ = self.compute_points(res_, dt_list) # Let's assume that sideways speed doesn't change
		pitch_ = [self.angles["pitch"]] * np.ones(len(roll_)) # It changes by some way 
		
		sideways_vel_ = [self.velocities["sideways"]] * \
						 np.ones(len(roll_)) # It seems be constant 

		self.update_all_and_log(res_, sideways_vel_, # velocities and 1 angle 
								longitude_, latitude_, height_, # position
								roll_, pitch_, dt_list) # 2 angles

	def compute_points(self, res, dt_list):
		forw_vel = res[:,0]
		down_vel = res[:,1]
		yaw = res[:,2]

		longitude_ = self.pos["longitude"] + forw_vel*np.cos(yaw)*dt_list
		latitude_ = self.pos["latitude"] + forw_vel*np.sin(yaw)*dt_list
		height_ = self.pos["height"] - down_vel*dt_list

		return (longitude_, latitude_, height_)

	def update_all_and_log(self, res, sideways_vel,
						   longitude, latitude, height,
						   roll, pitch, dt_list):
		# Add to arrays
		""" 
			Slicing example
			a = [(1,2,3)]

			b1 = np.array((1, 2, 3))
			b2 = np.array((4, 5, 6))
			b3 = np.array((7, 8, 9))
		
			b_tot = np.stack((b1, b2, b3), axis=-1)

			list(map(tuple, b_tot))
			>>> [(1, 4, 7), (2, 5, 8), (3, 6, 9)]
			>>> a + b_tot

		"""
		# Log
		self.trajectory += list(map(tuple, np.stack((longitude,
													 latitude, height),
													 axis=-1))) # trajectory history array		
		self.time_fly += list(Env.time + dt_list) # time history array
		self.vel_array += list(map(tuple, np.stack((res[:, 0], sideways_vel,
													res[:, 1]),
													axis=-1))) # velocities history array 
		
		self.angles_array += list(map(tuple, np.stack((res[:,2], roll,
													pitch),
													axis=-1))) # Angles history array 

		# Update
		self.pos["longitude"] = self.trajectory[-1][0]
		self.pos["latitude"] = self.trajectory[-1][1]
		self.pos["height"] = self.trajectory[-1][2]

		Env.time = self.time_fly[-1]		

		self.velocities["forward"] = self.vel_array[-1][0]
		self.velocities["sideways"] = self.vel_array[-1][1]
		self.velocities["down"] = self.vel_array[-1][2]


		self.angles["yaw"] = self.angles_array[-1][0]
		self.angles["roll"] = self.angles_array[-1][1] # Update of roll
		self.angles["pitch"] = self.angles_array[-1][2]

#		print("angels", self.angles["roll"])

	def compute_and_optimize_num_diff_eqs(self, y0, dt_list):
		""" Solves diff equations, corrects it if it is inappropriate """
		#print("Time_: ", Env.time)
		res_ = odeint(self.traj_init, y0, dt_list)

		# Is any velocity exceed limits?
		for i in range(res_.shape[0]):
			if res_[i,0] > 30: 
				res_[i,0] = 30
			if res_[i,0] < 1: # Different value?
				res_[i,0] = 1

			if res_[i,1] > 20:
				res_[i,1] = 20
			if res_[i,1] < 1:
				res_[i,1] = 1

			if abs(res_[i,2]) > np.pi:
				res_[i,2] = less_than_pi(res_[i,2])
		return res_



# !!! Initial values for all functions?
# Save the list of all used brakes combinations
# Upadate init conditions for all functions every time they are called

