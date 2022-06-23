import pygame
import random
import math
import numpy as np
import argparse

# Command line arguments
parser = argparse.ArgumentParser(description='Implements the RRT algorithm.')
parser.add_argument('-o', '--obstacles', type=bool, action=argparse.BooleanOptionalAction, metavar='', required=False, help='Obstacles on the map')
parser.add_argument('-n', '--nodes', type=int, metavar='', required=False, help='Maximum number of nodes')
parser.add_argument('-e', '--epsilon', type=float, metavar='', required=False, help='Step size')
parser.add_argument('-srn', '--show_random_nodes', type=bool, action=argparse.BooleanOptionalAction, metavar='', required=False, help='Show random nodes on screen')
parser.add_argument('-snn', '--show_new_nodes', type=bool, action=argparse.BooleanOptionalAction, metavar='', required=False, help='Show new nodes on screen')
args = parser.parse_args()

# Constants
WIDTH, HEIGHT = 640, 480
WINDOW = pygame.display.set_mode(size=(WIDTH, HEIGHT))
pygame.display.set_caption('RRT')
FPS = 60

# Colors 
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
BROWN = (189, 154, 122)

# RRT parameters
if not args.nodes == None:
	MAX_NODES = args.nodes
else:
	MAX_NODES = 5000 #  Default maximum number of nodes/vertices

if not args.epsilon == None:
	EPSILON = args.epsilon
else:
	EPSILON = 7.0 # Default step size

def draw_window():
	"""Draw the window all white."""
	WINDOW.fill(WHITE)

def draw_obstacles():
	"""Draw obstacles on the window.

	Parameters
	----------
	None

	Returns
	-------
	list
		All the rectangle obstacles.
	"""
	obstacles = []
	obstacle1 = pygame.Rect((WIDTH//2 + 90, HEIGHT//2 - 45, 90, 90))
	obstacle2 = pygame.Rect((WIDTH//2 - 180, HEIGHT//2 - 45, 90, 90))
	pygame.draw.rect(surface=WINDOW, color=RED, rect=obstacle1)
	pygame.draw.rect(surface=WINDOW, color=RED, rect=obstacle2)
	pygame.draw.rect(surface=WINDOW, color=BLACK, rect=obstacle1, width=2)
	pygame.draw.rect(surface=WINDOW, color=BLACK, rect=obstacle2, width=2)

	obstacles.append(obstacle1)
	obstacles.append(obstacle2)

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
	to the random generated node.

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
		distance = euclidean_distance(state, x_rand)
		distances.append(distance)

	x_near = tree[np.argmin(distances)]

	return x_near

def new_state(x_rand, x_near):
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

	Returns
	-------
	tuple
		Coordinate of the new node generated between the nearest
		and random nodes.	
	"""
	if euclidean_distance(x_near, x_rand) < EPSILON:
		# Keep that shortest distance from x_near to x_rand
		return x_rand
	else:
		px, py = x_rand[0] - x_near[0], x_rand[1] - x_near[1]
		theta = math.atan2(py, px)
		x_new = x_near[0] + EPSILON*math.cos(theta), x_near[1] + EPSILON*math.sin(theta)

		return x_new

def main(has_obstacles, show_random_nodes, show_new_nodes):
	clock = pygame.time.Clock()
	run = True
	tree = [] # Tree containing all the nodes/vertices
	x_init = WINDOW.get_rect().center # Initial node
	tree.append(x_init) # Append initial node
	draw_window()
	if has_obstacles:
		obstacles = draw_obstacles()	

	k = 0
	while run and k < MAX_NODES:
		# Make sure the loop runs at 60 FPS
		clock.tick(FPS)  
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		x_rand = generate_random_node() # Random node 
		x_near = nearest_neighbor(tree, x_rand) # Nearest neighbor to the random node
		x_new = new_state(x_rand, x_near) # New node
		tree.append(x_new)
	
		# Draw points and lines to visualization
		pygame.draw.circle(surface=WINDOW, color=BLUE, center=x_init, radius=3)

		if has_obstacles:
			collision_free = is_free(point=x_new, obstacles=obstacles, tree=tree) # Check collision
			if collision_free:
				if show_random_nodes:
					pygame.draw.circle(surface=WINDOW, color=GREEN, center=x_rand, radius=3)
				if show_new_nodes:
					pygame.draw.circle(surface=WINDOW, color=BROWN, center=x_new, radius=2)

				pygame.draw.line(surface=WINDOW, color=BLACK, start_pos=x_near, end_pos=x_new)
		else:
			if show_random_nodes:
				pygame.draw.circle(surface=WINDOW, color=GREEN, center=x_rand, radius=3)
			if show_new_nodes:
				pygame.draw.circle(surface=WINDOW, color=BROWN, center=x_new, radius=2)

			pygame.draw.line(surface=WINDOW, color=BLACK, start_pos=x_near, end_pos=x_new)

		pygame.display.update()
		
		k += 1

	pygame.quit()

if __name__ == '__main__':
	main(has_obstacles=args.obstacles, show_random_nodes=args.show_random_nodes, show_new_nodes=args.show_new_nodes)