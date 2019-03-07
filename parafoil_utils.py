import numpy as np
from scipy.integrate import odeint
from collections import UserDict # Can they be used to redefine some methods?
from IPython.core.debugger import set_trace


class VectorDict(UserDict): # Is it useful?

	""" Dict with maximum value restriction """
	
	def __init__(self, *args, speed_limit, **kwargs):
		self.speed_limit = speed_limit
		super().__init__(*args, **kwargs)
	
	def __setitem__(self, key, value):
		if value > abs(self.speed_limit):
			value = np.sign(value) * self.speed_limit
			print("speed limit: {0}".format(value))
					
		super().__setitem__(key, value)

def less_than_pi(a):
	"""Converts the angles to less
	   than pi one with a corresponding sign"""

	a_remainder = abs(a) % 2*np.pi
	if a_remainder > np.pi:
		if np.sign(a) < 0:
			return np.pi - (a_remainder % np.pi)
		else:
			return (a_remainder % np.pi) - np.pi
	else:
		return np.sign(a)*a_remainder
