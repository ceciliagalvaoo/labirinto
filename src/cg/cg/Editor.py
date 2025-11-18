import pygame
import sys

from .Maze import Maze
from .Utils.Csv import save_to_csv

class Editor:
    def __init__(self, maze_initial_configuration, map_file="culling_games/maps/default.csv",
                 resolution=720):
        self.maze = Maze(maze_initial_configuration, resolution)
        pygame.init()
        self.map_file = map_file
        self.running = False

    def update(self):
        self.handle_input()
        self.maze.draw()

    def handle_input(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                save_to_csv(self.map_file, self.maze.get_occupancy_grid())
            elif event.type == pygame.MOUSEBUTTONDOWN:
                cell_size = self.maze.cell_size
                x, y = event.pos
                grid_row = y // cell_size
                grid_col = x // cell_size
                if event.button == 1:  # Left click to place wall
                    if self.maze.get_cell((grid_row, grid_col)) == "f":
                        self.maze.set_cell((grid_row, grid_col), 'b')
                    else:
                        self.maze.set_cell((grid_row, grid_col), 'f')
                elif event.button == 3:  # Right click to set target
                    self.maze.set_cell((grid_row, grid_col), 't')
                elif event.button == 2:  # Middle click to set robot start
                    self.maze.set_cell((grid_row, grid_col), 'r')
        
    def run(self):
        self.running = True
        while self.running:
            self.update()
            pygame.display.flip()
        pygame.quit()
        sys.exit()
