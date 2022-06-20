import pygame

# Constants
WIDTH, HEIGHT = 640, 480
WINDOW = pygame.display.set_mode(size=(WIDTH, HEIGHT))
pygame.display.set_caption('RRT')
FPS = 60

# Colors 
WHITE = (255, 255, 255)

def draw_window():
	WINDOW.fill(WHITE)
	pygame.display.update()

def main():
	clock = pygame.time.Clock()
	run = True
	while run:
		# Makes sure the loop runs at 60 FPS
		clock.tick(FPS)  
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		draw_window()		

	pygame.quit()

if __name__ == '__main__':
	main()