import numpy as np
from scipy.integrate import odeint
from collections import UserDict # Can they be used to redefine some methods?
from IPython.core.debugger import set_trace
from parafoil_env import *
from parafoil_utils import *
from parafoil_params import *

	
class Parafoil(Parafoil_Params):
	"""Trajectory calculation tools"""

	def __init__(self):
		super().__init__() 
		self.offset = 0#0.0001 # For ODEs . ??? 
		print("Time: ", Env.time)
		
	def delta_sym(self):
		return (self.K_delta*self.delta["right"] + self.delta["left"])/2

	def delta_asym(self):
		return (self.K_delta*self.delta["right"]
			- self.delta["left"]
			+ self.D_delta)

	def lift(self):
		CL_DELTA_ = np.array(self.CL_delta)
		air_ro_ = np.array(Env.air_ro())
		SIZE_ = np.array(list(self.load_dims.values())[::-1]) # Get list frim dict
		air_speed_ = np.array(Env.air_speed(1))
    
		assert str(type(CL_DELTA_)) == "<class 'numpy.ndarray'>", "Wrong type"    
    
		return (air_ro_ * SIZE_ * (air_speed_**2)
				*(self.CL + CL_DELTA_*self.delta_sym())/2) # [0] scalar

	def drag(self): # 3D Vector
		CD_DELTA_ = np.array(self.Cd_delta)
		air_ro_ = np.array(Env.air_ro())
		SIZE_ = np.array(list(self.load_dims.values())[::-1]) # Get list from dict
		air_speed_ = np.array(Env.air_speed(1))
    
		assert str(type(CD_DELTA_)) == "<class 'numpy.ndarray'>", "Wrong type"    
    
		return (air_ro_ * SIZE_ * (air_speed_**2)
				*(self.Cd + CD_DELTA_*self.delta_sym())/2) # [0] scalar

	def guide(self):
		None
		
	def attack_angle(self): # Of arafoil
		w = self.velocities["down"]
		u = self.velocities["forward"]
		return np.arctan2(w, u) # Abs(w/u) < 1
	
	def diff_down_velocity(self, t):
		L = self.lift()[2] # 1D !!!
		D = self.drag()[2] # 1D !!!
		alpha = self.attack_angle()
		w = self.velocities["down"]
		u = self.velocities["forward"]

		assert w > 0, "Down speed is zero or negative or negative!"
		assert u > 0, "Forward speed is zero or negative!"

		roll_ = self.roll(t) # Can be done in one line but by 2 it's more clearer
		self.roll_init = roll_ # For next iteration
		return (
	        (-L*np.cos(alpha)-D*np.sin(alpha))/self.mass + 
	        (Env.g*np.cos(roll_) + u*self.diff_yaw(t)*np.sin(roll_))
	        )
	
	def roll(self, t): # Update initial conditions every update period only !!!

		assert self.angles["roll"] < np.pi/3, 'Roll exceeds the limits'

		delta_roll_ = self.K_PHI*self.delta_asym()*(1
					- np.exp(-t/self.T_PHI))

		roll_current_ = self.angles["roll"]
		
		diff_ = roll_current_ - delta_roll_ # It is array
		
		#print(roll_current_, diff_, delta_roll_, "Max: ", self.K_PHI*self.delta_asym(), "time: ", t)

 	   	# Change roll if it is needed
		if diff_>0 and (abs(roll_current_) < abs(diff_)-1e-2): #For a delay!!!
			roll_current_ = roll_current_ - abs(diff_)
			print("1: ", roll_current_, diff_, delta_roll_)
		elif diff_<0 and (abs(roll_current_) < abs(diff_)-1e-2): # For a delay!!! Compare with
			roll_current_ = roll_current_ + abs(diff_) 
			print("2: ", roll_current_, diff_, delta_roll_)
		
		if roll_current_>np.pi/3:
			roll_current_ = np.pi/3
		if roll_current_<-np.pi/3:
			roll_current_ = -np.pi/3

		return roll_current_

	def roll_vector(self, t_list): # To get an array of roll angles
		roll_ = []
		for t_ in t_list:
			roll_.append(self.roll(t_))
		return roll_
		
	def diff_roll(self, t):
		return self.K_PHI*self.delta_asym()*np.exp(-t/self.T_PHI)/self.T_PHI

	def diff_yaw(self, t):
		""" u != 0 roll t!= pi/2 """
		w = self.velocities["down"]
		u = self.velocities["forward"]

		assert w > 0, "Down speed is zero or negative!"
		assert u > 0, "Forward speed is zero or negative!"

		d = self.offset
		roll_ = self.roll(t)
		
		return (Env.g*np.tan(roll_)/(u+d)
				+ (w*self.diff_roll(t))/(u+d)
				/ (np.cos(roll_)+d))

	def diff_forward_velocity(self, t): # forward velocity can't be zero
	    L = self.lift()[0] # 1D !!!
	    alpha = self.attack_angle()
	    D = self.drag()[0] # 1D !!!
	    w = self.velocities["down"]

#	    assert w > 0, "Down speed is zero or negative!"

	    roll_ = self.roll(t)   # Can be done in one line but by 2 it's more clearer
	    
	    #return (
	    #    (L*np.sin(alpha)-D*np.sin(alpha))/self.mass)
	    
	    return (
	        (L*np.sin(alpha)-D*np.sin(alpha))/self.mass -
	        (w*self.diff_yaw(t)*np.sin(roll_))
	        )