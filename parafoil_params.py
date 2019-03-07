import numpy as np
from scipy.integrate import odeint
from collections import UserDict # Can they be used to redefine some methods?
from IPython.core.debugger import set_trace
from parafoil_env import *
from parafoil_utils import *


class Parafoil_Params():	
	"""Physical parameters of the load and the parafoil"""

	def __init__(self):
		self.load_dims = {
			"Ax": 0.6,
			"Ay": 0.4,
			"Az": 0.5
		} # [Meters^2]
		self.mass = 100
		
		self.K_PHI = 0.4 # Some aerodynamics coefficient [rad/s] for ROLL
		self.T_PHI = 0.3 # 0.994 s
		
		# Init or current? pos?
		self.pos = {
			"longitude": 0,
			"latitude": 0,
			"height": 1000
		} # Position  
		self.target = {
			"longitude": 10,
			"latitude": 20,
			"height": 30
		} # Target position
		
		self.angles = VectorDict(yaw=0,
								roll=0,
								pitch=np.pi/2, 
								speed_limit=np.pi) # What about yaw ??? 

		self.velocities = VectorDict(forward=9,
									sideways=0,
									down=4.7, 
									speed_limit=30) # Forward, Down velocities

		# Actuator control
		self.delta = {"left": 0, "right": 0} # parafoil chords length
		self.Cd = [2,4,5] # How to choose?
		self.Cd_delta = [0.2, 0.4, 0.5] # How to choose ?
		self.CL = [2,4,5] # How to choose ?
		self.CL_delta = [0.2, 0.4, 0.5] # How to choose ?
		self.D_delta = -0.141 # How to choose ?
		self.K_delta = 1.647 # How to choose?
