import random
import csv
import sys

def generate_maze(width, height):
    # Grid dimensions must be odd for this algorithm
    if width % 2 == 0:
        width -= 1
    if height % 2 == 0:
        height -= 1

    # Initialize grid with walls
    maze = [['b' for _ in range(width)] for _ in range(height)]

    # Stack for DFS
    stack = []

    # Start DFS from a random odd position
    start_x, start_y = (1, 1)
    maze[start_y][start_x] = 'f'
    stack.append((start_x, start_y))

    while stack:
        x, y = stack[-1]

        # Find unvisited neighbors (2 cells away)
        neighbors = []
        for dx, dy in [(0, 2), (0, -2), (2, 0), (-2, 0)]:
            nx, ny = x + dx, y + dy
            if 0 < nx < width - 1 and 0 < ny < height - 1 and maze[ny][nx] == 'b':
                neighbors.append((nx, ny))

        if neighbors:
            # Choose a random neighbor
            nx, ny = random.choice(neighbors)

            # Carve path to neighbor
            maze[ny][nx] = 'f'
            maze[y + (ny - y) // 2][x + (nx - x) // 2] = 'f'

            stack.append((nx, ny))
        else:
            # Backtrack
            stack.pop()

    # Set start and target
    maze[1][1] = 'r'
    # Center of an odd-dimensioned grid is floor(dim/2)
    center_y, center_x = height // 2, width // 2
    maze[center_y][center_x] = 't'

    return maze

def print_maze_csv(maze):
    writer = csv.writer(sys.stdout)
    writer.writerows(maze)

if __name__ == "__main__":
    # Generate a 29x29 maze, which is close to 30x30
    maze = generate_maze(29, 29)
    print_maze_csv(maze)
