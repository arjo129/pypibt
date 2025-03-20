import pygame
from pypibt import CollisionChecker

def draw_image(red_circle_pos, blue_circle_pos):
    """
    Draws the image with a blue track, red track, and circles.

    Args:
        red_circle_pos: Tuple (x, y) for the red circle's center.
        blue_circle_pos: Tuple (x, y) for the blue circle's center.
    """

    pygame.init()

    # Set up the window
    width, height = 400, 400
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Track with Circles")

    # Colors
    white = (255, 255, 255)
    blue = (135, 206, 250)  # Light blue
    red = (255, 0, 0)
    green = (0, 255, 0)

    # Track dimensions
    track_width = 50
    track_length = 100
    gap = 20

    # Calculate track positions
    horizontal_track_y = height // 2 - track_width // 2
    vertical_track_x = width // 2 - track_width // 2


    # Subdivide the horizontal blue track into rectangles with borders
    num_horizontal_rects = 8
    horizontal_rect_width = width // num_horizontal_rects
    blue_center_pos = []
    graph1_nodes = []
    for i in range(num_horizontal_rects):
        rect_x = i * horizontal_rect_width
        pygame.draw.rect(screen, blue, (rect_x, horizontal_track_y, horizontal_rect_width, track_width))
        pygame.draw.rect(screen, white, (rect_x, horizontal_track_y, horizontal_rect_width, track_width), 1)  # Border
        # Print the center of the circle
        blue_center_pos.append((rect_x + horizontal_rect_width // 2, horizontal_track_y + track_width // 2))
        print(f"Horizontal rectangle center: ({rect_x + horizontal_rect_width // 2}, {horizontal_track_y + track_width // 2})")

    # Subdivide the vertical red track into rectangles with borders
    num_vertical_rects = 8
    vertical_rect_height = height // num_vertical_rects
    red_center_pos = []
    for i in range(num_vertical_rects):
        rect_y = i * vertical_rect_height
        pygame.draw.rect(screen, red, (vertical_track_x, rect_y, track_width, vertical_rect_height))
        pygame.draw.rect(screen, white, (vertical_track_x, rect_y, track_width, vertical_rect_height), 1)  # Border
        red_center_pos.append((vertical_track_x + track_width // 2, rect_y + vertical_rect_height // 2))


    # Draw circles
    circle_radius = 10
    print(red_center_pos[red_circle_pos])
    pygame.draw.circle(screen, red, red_center_pos[red_circle_pos], circle_radius)
    pygame.draw.circle(screen, white, red_center_pos[red_circle_pos], circle_radius, 1)  # Border for red circle
    pygame.draw.circle(screen, blue, blue_center_pos[blue_circle_pos], circle_radius)
    pygame.draw.circle(screen, white, blue_center_pos[blue_circle_pos], circle_radius, 1)  # Border for blue circle

    # Update the display
    pygame.display.flip()

    # Event loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == "__main__":
    red_circle_pos = 2  # Example position
    blue_circle_pos = 3 # Example position
    draw_image(red_circle_pos, blue_circle_pos)