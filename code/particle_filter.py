
import numpy as np
import scipy
import math

# class for a single particle
class Particle:
	# postion x, position y, angle delta, weight w
	def __init__(self, x, y, d, w_prev, w):
		self.x = x
		self.y = y
		self.d = d
		self.w_prev = w_prev
		self.w = w


class ParticleFilter:
	def __init__(self):
		# create 500 randomly distributed particles
		self.pList = []
		angle = [0, math.pi/2, math.pi, 3*math.pi/2, 2*math.pi]
		for i in range(500):
			x = np.random.uniform(0.1,2.9)
			y = np.random.uniform(0.1,2.9)
			d = np.random.choice(angle)
			w_prev = 1/500.0
			w = 0
			p = Particle(x,y,d,w_prev,w)

			self.pList.append(p)
			print("particle {} created".format(i))

	# particle movement
	def move(self, move):
		angle = [0, math.pi/2, math.pi, 3*math.pi/2, 2*math.pi]
		
		if move == "forward":
			for i in range(500):
				new_x = self.pList[i].x + ( (0.2 + np.random.normal(0, 0.1)) * math.cos(self.pList[i].d) )
				new_y = self.pList[i].y + ( (0.2 + np.random.normal(0, 0.1)) * math.sin(self.pList[i].d) )
				if 0< new_x <3:
					self.pList[i].x = new_x
				if 0< new_y <3:
					self.pList[i].y = new_y

		if move == "right":
			for i in range(500):
				new_d = angle_adjust(self.pList[i].d - math.pi / 2 + np.random.normal(0, 0.1))
				self.pList[i].d = new_d

		if move == "left":
			for i in range(500):
				new_d = angle_adjust(self.pList[i].d + math.pi / 2 + np.random.normal(0, 0.1))
				self.pList[i].d = new_d

def angle_adjust(angle):
  if angle > math.pi:
      return angle - 2 * math.pi
  elif angle < -math.pi:
      return 2 * math.pi  + angle
  return angle
