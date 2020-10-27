import argparse
import numpy as np

def main():
	np.random.seed(0)
	# States
	x = np.zeros(2)
	x_min = -5
	x_max = -x_min
	v_min = -1
	v_max = -v_min

	# Actions
	a = 0

	# System Dynamics
	A = np.array([[0, 1],[0,1]])
	B = np.array([[0],[1]])
	C = np.eye(2)
	D = np.array([[0],[0]])

	def step(x, u):
		return np.dot(A, x) + np.dot(B, u)

	# Goal State
	s_f = np.array([[0],[0]])
	print(s_f)
	# Goal Space
	allowable_dx = 0.5
	allowable_dv = 0.1
	# def goal_reached(s_curr):
	# 	if (np.abs(s_curr[0,0] - s_f[0,0]) < allowable_dx and np.abs()):

	# Initial State
	s_i = np.zeros((2,1)) 

	s_i[0,0] = np.random.uniform(x_min, x_max)
	s_i[1,0] = np.random.uniform(v_min, v_max)
	print(s_i)

	# Random Sample
	def random_sample():
		s_rand = np.zeros((2,1)) 
		s_rand[0,0] = np.random.uniform(x_min, x_max)
		s_rand[1,0] = np.random.uniform(v_min, v_max)
		return s_rand


	V = [s_i]
	E = []

	# Construct the graph
	def find_nearest_vertex(s_rand):
		s_near = None
		nearest_distance = np.inf()
		for i in range(len(V)):
			curr_distance = np.linalg.norm(V[i] - s_rand)
			if curr_distance < nearest_distance:
				nearest_distance = curr_distance
				s_near = V[i]
		return s_near

	def drive(s_rand, s_near):
		num_iters = 1 # number of max steps to drive


		a = np.clip(, -1, 1)
		step(s_near, )





	complete = False
	while not complete:
		s_rand = random_sample()
		s_near = find_nearest_vertex(s_rand)



		




if __name__ == "__main__":
	main()