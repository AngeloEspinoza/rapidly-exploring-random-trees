import pygame
import random
import math
import numpy as np

class Graph():
	"""
	A class for the Rapidly-exploring Random Tree (RRT).
	
	Attributes
	----------
	start : tuple
		Initial position of the tree in X and Y respectively.
	goal : tuple
		End position of the tree in X and Y respectively.
	map_dimensions : tuple
		Map width and height in pixels.
	"""

	def __init__(self, start, goal, map_dimensions, epsilon):
		self.x_init = start
		self.x_goal = goal

		self.WIDTH, self.HEIGHT = map_dimensions
		self.MAX_NODES = 100
		self.EPSILON = epsilon

		self.obstacles = None
		self.is_goal_reached = False

		# Colors 
		self.WHITE = (255, 255, 255)
		self.BLACK = (0, 0, 0)
		self.RED = (255, 0, 0)
		self.GREEN = (0, 255, 0)
		self.BLUE = (0, 0, 255)
		self.BROWN = (189, 154, 122)
		self.YELLOW = (255, 255, 0)
		self.TURQUOISE = (64, 224, 208)
		self.FUCSIA = (255, 0, 255)

	def is_free(self, point, obstacles, tree):
		"""Checks whether a node is colliding with an obstacle or not.

		When dealing with obstacles it is necessary to check 
		for the collision with them from the generated node.

		Parameters
		----------
		point : tuple
			Point to be checked.
		obstacles : pygame.Rect
			Rectangle obstacle.
		tree : list
			Tree containing all the coordinate nodes.

		Returns
		-------
		bool
		"""
		for obstacle in obstacles:
			if obstacle.collidepoint(point):
				tree.remove(point)
				return False

		return True

	def generate_random_node(self, ):
		"""Generates a random node on the screen.

		The x and y coordinate is generated given an uniform
		distribution of the size of the screen width and height.

		Parameters
		----------
		None

		Returns
		-------
		tuple
			Coordinates of the random node. 
		"""
		self.x_rand = random.uniform(0, self.WIDTH), random.uniform(0, self.HEIGHT)

		return self.x_rand

	def euclidean_distance(self, p1, p2):
		"""Euclidean distance between two points.

		Parameters
		----------
		p1 : int
			Start point.
		p2 : int 
			End point.

		Returns
		-------
		float
			Euclidean distance metric.
		"""
		return  math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2) 

	def nearest_neighbor(self, tree, x_rand):
		"""Returns the index of the nearest neighbor.
		
		The nearest neighbor from all the nodes in the tree
		to the randomly generated node.

		Parameters
		----------
		tree : list
			Tree containing all the coordinate nodes.
		x_rand : tuple 
			Coordinate of the random node generated.

		Returns
		-------
		tuple
			Nearest node to the random node generated.	
		"""

		distances = []

		for state in tree:
			distance = self.euclidean_distance(state, x_rand)
			distances.append(distance)
			
		# Index of the minimum distance to the generated random node
		self.min_distance = np.argmin(distances) 
		x_near = tree[self.min_distance]

		return x_near

	def new_state(self, x_rand, x_near, x_goal):
		"""Advances a small step (self.EPSILON) towards the random node.
		
		Takes small step (self.EPSILON) from the nearest node to the
		random node, if the distance is greater than the small step.
		Otherwise, takes the smallest distance computed by the metric
		distance.

		Parameters
		----------
		x_rand : tuple
			Coordinate of the random node generated.
		x_near : tuple 
			Coordinate of the nearest neighbor node.
		x_goal : tuple
			Coordinate of the goal node.

		Returns
		-------
		tuple
			Coordinate of the new node generated between the nearest
			and random nodes.	
		"""
		if self.euclidean_distance(x_near, x_rand) < self.EPSILON:
			if abs(x_rand[0] - x_goal[0]) < self.EPSILON and abs(x_rand[1] - x_goal[1]) < self.EPSILON: # Check if goal is reached
				self.is_goal_reached = True

			# Keep that shortest distance from x_near to x_rand
			return x_rand
		else:
			px, py = x_rand[0] - x_near[0], x_rand[1] - x_near[1]
			theta = math.atan2(py, px)
			x_new = x_near[0] + self.EPSILON*math.cos(theta), x_near[1] + self.EPSILON*math.sin(theta) 

			if abs(x_new[0] - x_goal[0]) < self.EPSILON and abs(x_new[1] - x_goal[1]) < self.EPSILON: # Check if goal is reached
				self.is_goal_reached = True

			return x_new

	def generate_parents(self, values, parent):
		"""Generates a list of parents and their children.
		
		Sets up a list of the parents and its corresponding
		children of the tree given a value and the value 
		of the nearest neighbor.

		Parameters
		----------
		values : list
			Collection of values of the assigned x_new node.
		parent : list
			Collection of parents to be fulfilled given its
			correspondant x_near value.

		Returns
		-------
		list
			Ordered collection of the parents.
		"""
		parent_value = values[self.min_distance] # Value nearest node
		parent_index = len(parent) # Used to be the index of the parent list
		parent.insert(parent_index, parent_value)

		if self.is_goal_reached:
			# Insert in the very last index the last value recorded plus one
			parent.insert(parent_index+1, values[-1]+1)

		return parent

	def draw_random_node(self, map_):
		"""Draws the x_rand node."""
		pygame.draw.circle(surface=map_, color=self.GREEN, 
			center=self.x_rand, radius=3)

	def draw_new_node(self, map_):
		"""Draws the x_near node."""
		pygame.draw.circle(surface=WINDOW, color=self.BROWN,
			center=x_new, radius=2)

	def draw_initial_node(self, map_):
		"""Draws the x_init node."""
		pygame.draw.circle(surface=map_, color=self.BLUE, 
			center=self.x_init, radius=4)

	def draw_goal_node(self, map_):
		"""Draws the x_goal node."""
		pygame.draw.circle(surface=map_, color=self.RED, 
			center=self.x_goal, radius=4)

	def draw_local_planner(self, p1, p2, map_):
		"""Draws the local planner from node to node."""
		pygame.draw.line(surface=map_, color=self.BLACK,
			start_pos=p1, end_pos=p2)