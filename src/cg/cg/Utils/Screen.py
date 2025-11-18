import pygame
import os

max_screen_width, max_screen_height = 800, 600  # Adjust if needed

from ament_index_python.packages import get_package_share_directory

def generate_screen(maze, resolution):
    grid_size = resolution // len(maze)
    screen = pygame.display.set_mode(size=(resolution, resolution))
    pygame.display.set_caption("Culling Games")
    return screen, grid_size

def load_images_from_directory(directory=None):
    if directory is None:
        pkg_share_dir = get_package_share_directory('cg')
        directory = os.path.join(pkg_share_dir, 'lobotomy_kaisen')
    images = []
    for filename in os.listdir(directory):
        if filename.endswith((".png", ".jpg", ".jpeg", ".bmp", ".gif")):
            image_path = os.path.join(directory, filename)
            images.append(pygame.image.load(image_path))
    return images

# Display a random image centered on the screen
def display_image(screen, image):
    screen_rect = screen.get_rect()
    scaled_image = pygame.transform.scale(image, (screen_rect.width, screen_rect.height))
    image_rect = scaled_image.get_rect()
    image_rect.center = screen_rect.center
    
    screen.fill((0, 0, 0))  # Fill the screen with a black background
    screen.blit(scaled_image, image_rect)
    pygame.display.flip()
    return screen
