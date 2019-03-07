import numpy as np
from scipy.integrate import odeint
from collections import UserDict # Can they be used to redefine some methods?
from IPython.core.debugger import set_trace


class Env():
	"""Global environment characteristics"""

	time = 0 # Current
	g = 9.81 
	
	@classmethod # Can be called without object creation
	def get_time(self):
		return self.time
	
	@classmethod
	def air_ro(self):
		return 1.2 # Finish it
		
	@classmethod
	def air_speed(self, speed): # Upgrade it
		return [2, 4, 5] # Wrong

