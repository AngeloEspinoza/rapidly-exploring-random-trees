import pygame
import random
import math
import numpy as np
import argparse
import time
import sys 

# Command line arguments
parser = argparse.ArgumentParser(description='Implements the RRT algorithm.')
parser.add_argument('-o', '--obstacles', type=bool, action=argparse.BooleanOptionalAction, metavar='', required=False, help='Obstacles on the map')
parser.add_argument('-n', '--nodes', type=int, metavar='', required=False, help='Maximum number of nodes')
parser.add_argument('-e', '--epsilon', type=float, metavar='', required=False, help='Step size')
parser.add_argument('-init', '--x_init', nargs='+', type=int, metavar='', required=False, help='Initial node position in X and Y respectively')
parser.add_argument('-goal', '--x_goal', nargs='+', type=int, metavar='', required=False, help='Goal node position in X and Y respectively')
parser.add_argument('-srn', '--show_random_nodes', type=bool, action=argparse.BooleanOptionalAction, metavar='', required=False, help='Show random nodes on screen')
parser.add_argument('-snn', '--show_new_nodes', type=bool, action=argparse.BooleanOptionalAction, metavar='', required=False, help='Show new nodes on screen')
args = parser.parse_args()

# Constants
WIDTH, HEIGHT = 640, 480
WINDOW = pygame.display.set_mode(size=(WIDTH, HEIGHT))
pygame.display.set_caption('RRT')
FPS = 120

# Colors 
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
BROWN = (189, 154, 122)
GRAY = (105, 105, 105)

# RRT parameters
MAX_NODES = args.nodes if not args.nodes is None else 5000 # Default maximum number of nodes/vertices
EPSILON = args.epsilon if not args.epsilon is None else 7.0 # Default step size
obstacles = []

def draw_window():
	"""Draw the window all white."""
	WINDOW.fill(WHITE)

# def draw_obstacles():
# 	"""Draw obstacles on the window.

# 	Parameters
# 	----------
# 	None

# 	Returns
# 	-------
# 	list
# 		All the rectangle obstacles.
# 	"""
# 	obstacles = []
# 	obstacle1 = pygame.Rect((WIDTH//2 + 90, HEIGHT//2 - 45, 90, 90))
# 	obstacle2 = pygame.Rect((WIDTH//2 - 180, HEIGHT//2 - 45, 90, 90))
# 	pygame.draw.rect(surface=WINDOW, color=RED, rect=obstacle1)
# 	pygame.draw.rect(surface=WINDOW, color=RED, rect=obstacle2)

# 	# Just decoration borders in black
# 	pygame.draw.rect(surface=WINDOW, color=BLACK, rect=obstacle1, width=2)
# 	pygame.draw.rect(surface=WINDOW, color=BLACK, rect=obstacle2, width=2)

# 	obstacles.append(obstacle1)
# 	obstacles.append(obstacle2)

# 	return obstacles

def make_obstacles_T(initial_point):
	"""
	Given a initial point, it makes a obstacle with shape of T.
	
	Parameters
	----------
	initial_point : tuple
		X and Y coordinates, starting from the top-left most part where
		the obstacle will be placed.
	
	Returns
	-------
	list
		A collection of sides composing the T obstacle.			
	"""
	x, y = initial_point[0], initial_point[1]
	width, height = 50, 150

	side1 = pygame.Rect(x, y, height, width)
	side2 = pygame.Rect((x+height//2) - width//2, y, width, height)

	obstacle = [side1, side2]

	return obstacle

def make_obstacles_L(initial_point):
	"""
	Given a initial point, it makes a obstacle with shape of L.
	
	Parameters
	----------
	initial_point : tuple
		X and Y coordinates, starting from the top-left most part where
		the obstacle will be placed.
	
	Returns
	-------
	list
		A collection of sides composing the L obstacle.
	"""	
	x, y = initial_point[0], initial_point[1]
	width, height = 50, 150

	side1 = pygame.Rect(x, y, width, height)
	side2 = pygame.Rect(x, y+height-width, height, width)

	obstacle = [side1, side2]

	return obstacle

def make_obstacles():
	"""Generate the obstacles to be placed on the final map."""

	obstacle1 = make_obstacles_T(initial_point=(350, 200))
	obstacle2 = make_obstacles_L(initial_point=(150, 20))

	obstacles.append(obstacle1)
	obstacles.append(obstacle2)

	return obstacles

def draw_obstacles():
	"""Draw each side of the obstacles."""
	obstacles = []
	obs = make_obstacles()

	for obstacle in obs:
		for side in obstacle:
			pygame.draw.rect(surface=WINDOW, color=GRAY,
				rect=side)
			obstacles.append(side)

	return obstacles

def is_free(point, obstacles, tree):
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

def generate_random_node():
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
	x, y = random.uniform(0, WIDTH), random.uniform(0, HEIGHT)

	return x, y

def euclidean_distance(p1, p2):
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

def nearest_neighbor(tree, x_rand):
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
	global min_distance

	distances = []

	for state in tree:
		distance = euclidean_distance(state, x_rand)
		distances.append(distance)
		
	# Index of the minimum distance to the generated random node
	min_distance = np.argmin(distances) 
	x_near = tree[min_distance]

	return x_near

def new_state(x_rand, x_near, x_goal):
	"""Advances a small step (EPSILON) towards the random node.
	
	Takes small step (EPSILON) from the nearest node to the
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
	global is_goal_reached

	if euclidean_distance(x_near, x_rand) < EPSILON:
		if abs(x_rand[0] - x_goal[0]) < EPSILON and abs(x_rand[1] - x_goal[1]) < EPSILON: # Check if goal is reached
			is_goal_reached = True

		# Keep that shortest distance from x_near to x_rand
		return x_rand
	else:
		px, py = x_rand[0] - x_near[0], x_rand[1] - x_near[1]
		theta = math.atan2(py, px)
		x_new = x_near[0] + EPSILON*math.cos(theta), x_near[1] + EPSILON*math.sin(theta) 

		if abs(x_new[0] - x_goal[0]) < EPSILON and abs(x_new[1] - x_goal[1]) < EPSILON: # Check if goal is reached
			is_goal_reached = True

		return x_new

def generate_parents(values, parent):
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
	parent_value = values[min_distance] # Value nearest node
	parent_index = len(parent) # Used to be the index of the parent list
	parent.insert(parent_index, parent_value)

	if is_goal_reached:
		# Insert in the very last index the last value recorded plus one
		parent.insert(parent_index+1, values[-1]+1)

	return parent

def main(has_obstacles, show_random_nodes, show_new_nodes):
	global is_goal_reached

	clock = pygame.time.Clock()
	run = True
	is_goal_reached = False
	is_simulation_finished = False
	tree = [] # Tree list containing all the nodes/vertices
	parent = [] # Parent list of each each node/vertex
	values = [] # Values list of each node/vertex
	x_init = args.x_init if not args.x_init is None else WINDOW.get_rect().center # Initial node
	x_goal = args.x_goal if not args.x_goal is None else (540, 380) # Goal node
	tree.append(x_init) # Append initial node
	parent.append(0) # Append initial parent
	draw_window()
	if has_obstacles:
		obstacles = draw_obstacles()	

	k = 0
	node_value = 0
	iteration = 0
	
	while run and k < MAX_NODES:
		# Make sure the loop runs at 60 FPS
		clock.tick(FPS)  
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		x_rand = generate_random_node() # Random node 
		x_near = nearest_neighbor(tree, x_rand) # Nearest neighbor to the random node
		x_new = new_state(x_rand, x_near, x_goal) # New node
		tree.append(x_new)

		# Every n iterations bias the RRT
		if iteration%10 == 0:
			x_rand = x_goal

		iteration += 1

		# Draw points and lines to visualization
		pygame.draw.circle(surface=WINDOW, color=BLUE, center=x_init, radius=3)
		pygame.draw.circle(surface=WINDOW, color=RED, center=x_goal, radius=3)

		if has_obstacles and not is_simulation_finished:
			collision_free = is_free(point=x_new, obstacles=obstacles, tree=tree) # Check collision
			
			if collision_free:
				# Append current node value and place it in the parent list 
				values.append(node_value)
				parent = generate_parents(values, parent)

				if show_random_nodes:
					pygame.draw.circle(surface=WINDOW, color=GREEN, center=x_rand, radius=3)
				if show_new_nodes:
					pygame.draw.circle(surface=WINDOW, color=BROWN, center=x_new, radius=2)

				pygame.draw.line(surface=WINDOW, color=BLACK, start_pos=x_near, end_pos=x_new)
				node_value += 1 # Increment the value for the next randomly generated node

				if is_goal_reached:
					pygame.draw.line(surface=WINDOW, color=BLACK, start_pos=x_new, end_pos=x_goal)
					is_simulation_finished = True
					print(iteration)
		elif not is_simulation_finished:
			# Append current node value and place it in the parent list 
			values.append(node_value)
			parent = generate_parents(values, parent)


			if show_random_nodes:
				pygame.draw.circle(surface=WINDOW, color=GREEN, center=x_rand, radius=3)
			if show_new_nodes:
				pygame.draw.circle(surface=WINDOW, color=BROWN, center=x_new, radius=2)

			pygame.draw.line(surface=WINDOW, color=BLACK, start_pos=x_near, end_pos=x_new)
			node_value += 1 # Increment value for the next randomly generated node 

			if is_goal_reached:
				pygame.draw.line(surface=WINDOW, color=BLACK, start_pos=x_new, end_pos=x_goal)
				is_simulation_finished = True
				print(iteration)

		pygame.display.update()
		k += 1

	pygame.quit()
	sys.exit()

if __name__ == '__main__':
	main(has_obstacles=args.obstacles, show_random_nodes=args.show_random_nodes, show_new_nodes=args.show_new_nodes)