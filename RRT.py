import pygame
import environment
import graph 
import argparse
import sys

# Command line arguments
parser = argparse.ArgumentParser(description='Implements the RRT algorithm for path planning.')
parser.add_argument('-o', '--obstacles', type=bool, action=argparse.BooleanOptionalAction,
	metavar='', required=False, help='Obstacles on the map')
parser.add_argument('-n', '--nodes', type=int, metavar='', required=False,
	help='Maximum number of nodes')
parser.add_argument('-e', '--epsilon', type=float, metavar='', required=False, help='Step size')
parser.add_argument('-init', '--x_init', nargs='+', type=int, metavar='', required=False,
	help='Initial node position in X and Y respectively')
parser.add_argument('-goal', '--x_goal', nargs='+', type=int, metavar='', required=False,
	help='Goal node position in X and Y respectively')
parser.add_argument('-srn', '--show_random_nodes', type=bool, action=argparse.BooleanOptionalAction,
	metavar='', required=False, help='Show random nodes on screen')
parser.add_argument('-snn', '--show_new_nodes', type=bool, action=argparse.BooleanOptionalAction,
	metavar='', required=False, help='Show new nodes on screen')
parser.add_argument('-bp', '--bias_percentage', type=int, metavar='', required=False,
	help='Amount of bias the RRT from 1 to 100')
args = parser.parse_args()

# Constants
MAP_DIMENSIONS = 640, 480

# RRT parameters
MAX_NODES = args.nodes if args.nodes is not None else 5000 # Maximum number of nodes/vertices
EPSILON = args.epsilon if args.epsilon is not None else 7.0 # Step size

# Initial and final position of the robot
x_init = tuple(args.x_init) if args.x_init is not None else (50, 50) # Initial node
x_goal = tuple(args.x_goal) if args.x_goal is not None else (540, 380) # Goal node

# Instantiating the environment and the graph
environment_ = environment.Environment(map_dimensions=MAP_DIMENSIONS)
graph_ = graph.Graph(start=x_init, goal=x_goal, 
		map_dimensions=MAP_DIMENSIONS, epsilon=EPSILON)

def main():
	clock = pygame.time.Clock()
	run = True
	is_goal_reached = False
	is_simulation_finished = False
	tree = [] # Tree list containing all the nodes/vertices
	parent = [] # Parent list of each each node/vertex
	values = [] # Values list of each node/vertex
	tree.append(x_init) # Append initial node
	parent.append(0) # Append initial parent
	obstacles = environment_.draw_obstacles() if args.obstacles else []

	k = 0
	node_value = 0
	iteration = 0
	bias_percentage = 11 - args.bias_percentage//10
	print(bias_percentage)
	
	while run and k < MAX_NODES:
		# Make sure the loop runs at 60 FPS
		clock.tick(environment_.FPS)  
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		x_rand = graph_.generate_random_node() # Random node 
		x_near = graph_.nearest_neighbor(tree, x_rand) # Nearest neighbor to the random node
		x_new = graph_.new_state(x_rand, x_near, x_goal) # New node
		tree.append(x_new)

		# Every n iterations bias the RRT
		if iteration%10 == 0:
			x_rand = x_goal

		iteration += 1

		# Draw points and lines to visualization
		graph_.draw_initial_node(map_=environment_.map)
		graph_.draw_goal_node(map_=environment_.map)

		if not is_simulation_finished:
			collision_free = graph_.is_free(point=x_new, obstacles=obstacles, tree=tree) # Check collision
			
			if collision_free:
				# Append current node value and place it in the parent list 
				values.append(node_value)
				parent = graph_.generate_parents(values, parent)

				if args.show_random_nodes:
					graph_.draw_random_node(map_=environment_.map)		
				if args.show_new_nodes:
					graph_.draw_new_node(map_=environment_.map, n=x_new)

				graph_.draw_local_planner(p1=x_near, p2=x_new, map_=environment_.map)

				node_value += 1 # Increment the value for the next randomly generated node

				if graph_.is_goal_reached:
					graph_.draw_local_planner(p1=x_new, p2=x_goal, map_=environment_.map)
					is_simulation_finished = True

		pygame.display.update()
		k += 1

	pygame.quit()
	sys.exit()

if __name__ == '__main__':
	main()