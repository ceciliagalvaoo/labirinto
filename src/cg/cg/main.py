import rclpy
import threading
import os
import random
import sys
import argparse
import tempfile
import csv
import atexit
from ament_index_python.packages import get_package_share_directory

from .Utils.Csv import load_from_csv
from .Game import Game
from .Editor import Editor

# Helper function to generate a maze (copied from generate_maze.py)
def generate_maze_grid(width, height):
    if width % 2 == 0:
        width -= 1
    if height % 2 == 0:
        height -= 1

    maze = [['b' for _ in range(width)] for _ in range(height)]
    stack = []
    start_x, start_y = (1, 1)
    maze[start_y][start_x] = 'f'
    stack.append((start_x, start_y))

    while stack:
        x, y = stack[-1]
        neighbors = []
        for dx, dy in [(0, 2), (0, -2), (2, 0), (-2, 0)]:
            nx, ny = x + dx, y + dy
            if 0 < nx < width - 1 and 0 < ny < height - 1 and maze[ny][nx] == 'b':
                neighbors.append((nx, ny))

        if neighbors:
            nx, ny = random.choice(neighbors)
            maze[ny][nx] = 'f'
            maze[y + (ny - y) // 2][x + (nx - x) // 2] = 'f'
            stack.append((nx, ny))
        else:
            stack.pop()

    maze[1][1] = 'r'
    center_y, center_x = height // 2, width // 2
    maze[center_y][center_x] = 't'
    return maze


def game():
    parser = argparse.ArgumentParser(description='Culling Games Maze Runner')
    parser.add_argument('--map', type=str, help='Name of the map file to load from the maps directory (e.g., "test.csv")')
    parser.add_argument('--generate', action='store_true', help='Generate a new random maze and use it immediately.')
    args, unknown = parser.parse_known_args()

    rclpy.init(args=[arg for arg in sys.argv if arg not in ('-h', '--help')])
    
    share_dir = get_package_share_directory('cg')
    maps_dir = os.path.join(share_dir, 'maps')
    
    map_path = None
    temp_map_file = None

    if args.generate:
        print("Generating a new random maze...")
        maze_grid = generate_maze_grid(29, 29) # Use 29x29 for consistency
        
        temp_map_file = tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False)
        writer = csv.writer(temp_map_file)
        writer.writerows(maze_grid)
        temp_map_file.close()
        
        map_path = temp_map_file.name
        print(f"Generated maze saved to temporary file: {map_path}")
        atexit.register(os.remove, temp_map_file.name) # Register cleanup
    elif args.map:
        map_name = args.map
        if not map_name.endswith('.csv'):
            map_name += '.csv'
        map_path = os.path.join(maps_dir, map_name)
    else:
        available_maps = [f for f in os.listdir(maps_dir) if f.endswith('.csv')]
        if not available_maps:
            print("Error: No map files (.csv) found in the maps directory.", file=sys.stderr)
            return
        map_name = random.choice(available_maps)
        map_path = os.path.join(maps_dir, map_name)
    
    if not os.path.exists(map_path):
        print(f"Error: Map file not found at {map_path}", file=sys.stderr)
        return

    game = Game(map_path)
    thread = threading.Thread(target=rclpy.spin, args=(game,))
    thread.start()

    try:
        game.run()
    except KeyboardInterrupt:
        game.get_logger().info('KeyboardInterrupt received, shutting down.')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        thread.join()

def editor():
    share_dir = get_package_share_directory('cg')
    map_path = os.path.join(share_dir, 'maps', 'default.csv')
    editor = Editor(load_from_csv(map_path))
    editor.run()
