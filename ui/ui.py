#!/usr/bin/env python3
import pygame
import sys
from Map import Map


# Initialize Pygame
pygame.init()

# Set up the display
screen = pygame.display.set_mode((1024, 600))
pygame.display.set_caption('Image Clicker')

# Load the images
image_path = 'table.png'
image = pygame.image.load(image_path)
image = pygame.transform.scale(image, screen.get_size())

robot_image_path = 'robot.png'
robot_image = pygame.image.load(robot_image_path)

# Create the Map instance
map = Map(400, 400, 90)


# Function to draw a green rectangle border in the top left and bottom right corners
def draw_green_rectangle_border(surface, width, height):
    border_width = 10
    rectangle_color = (0, 255, 0)
    rectangle_position_top_left = (0, 0)  # Top left corner
    pygame.draw.rect(surface, rectangle_color, (*rectangle_position_top_left, width, height), border_width)

    rectangle_position_bottom_right = (surface.get_width() - width, surface.get_height() - height)
    pygame.draw.rect(surface, rectangle_color, (*rectangle_position_bottom_right, width, height), border_width)


# Functions to execute when clicking inside the green rectangles
def green_top_left():
    print("Clicked inside the TOP LEFT green rectangle!")
    robot_x = rectangle_width // 2 - robot_image.get_width() // 2
    robot_y = rectangle_height // 2 - robot_image.get_height() // 2
    rotated_robot_image = pygame.transform.rotate(robot_image, -90)
    screen.blit(rotated_robot_image, (robot_x, robot_y))

    map.goto(1000, 800, 0)
    pixel_x, pixel_y = map.getSimulationPos()
    pygame.draw.circle(screen, (255, 0, 0), (pixel_x, pixel_y), 25)
    pygame.display.flip()
    return 'top_left'

def green_bottom_right():
    print("Clicked inside the BOTTOM RIGHT green rectangle!")
    robot_x = screen_width - rectangle_width // 2 - robot_image.get_width() // 2
    robot_y = screen_height - rectangle_height // 2 - robot_image.get_height() // 2
    rotated_robot_image = pygame.transform.rotate(robot_image, 90)
    screen.blit(rotated_robot_image, (robot_x, robot_y))

    map.goto(2000, 1200, 90)
    pixel_x, pixel_y = map.getSimulationPos()
    pygame.draw.circle(screen, (255, 0, 0), (pixel_x, pixel_y), 25)
    pygame.display.flip()
    return 'bottom_right'




# Main loop
rectangle_width = 160
rectangle_height = 140
running = True
clicked_rectangle = None

# Draw the image
screen.blit(image, (0, 0))

# Draw the green rectangle borders
draw_green_rectangle_border(screen, rectangle_width, rectangle_height)

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Exit on pressing the ESC key
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False

        # Check if the click is inside the green rectangles and execute the corresponding function
        if event.type == pygame.MOUSEBUTTONDOWN:
            x, y = event.pos
            screen_width, screen_height = screen.get_size()
            if 0 <= x <= rectangle_width and 0 <= y <= rectangle_height:
                clicked_rectangle = green_top_left()
            elif screen_width - rectangle_width <= x <= screen_width and screen_height - rectangle_height <= y <= screen_height:
                clicked_rectangle = green_bottom_right()

    pygame.display.flip()

# Clean up and exit
pygame.quit()
sys.exit()


