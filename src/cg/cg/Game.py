import pygame
import sys
import threading

import rclpy
from cg_interfaces.srv import MoveCmd, GetMap, Reset
from cg_interfaces.msg import RobotSensors
from rclpy.node import Node

from .Maze import Maze
from .Robot import Robot
from .Utils.Finders import find
import os
import random
from ament_index_python.packages import get_package_share_directory

from .Utils.Grid import flatten
from .Utils.Csv import load_from_csv


class Game(Node):
    def __init__(self, map_path, resolution=720):
        super().__init__("Culling_Games")
        self.map_path = map_path
        self.resolution = resolution
        
        self.move_cmd_service = self.create_service(MoveCmd, 'move_command', self.handle_move_cmd)
        self.get_map_service = self.create_service(GetMap, 'get_map', self.handle_map_request)
        self.reset_service = self.create_service(Reset, 'reset', self.handle_reset_request)
        
        self.publisher_ = self.create_publisher(RobotSensors, '/culling_games/robot_sensors', 10)
        self.timer = self.create_timer(0.5, self.publish_sensor_data)
        
        initial_maze_config = load_from_csv(self.map_path)
        self.maze = Maze(initial_maze_config, self.resolution)
        self.robot = Robot(self.maze)
        
        self.running = False
        self.win = False
        self.win_handled = False
        
        self.reset_requested = False
        self.state_lock = threading.Lock()

    def handle_reset_request(self, request, response):
        with self.state_lock:
            self.get_logger().info(f'Reset request received: is_random={request.is_random}, map_name={request.map_name}')
            if request.is_random:
                maps_dir = os.path.dirname(self.map_path)
                available_maps = [f for f in os.listdir(maps_dir) if f.endswith('.csv')]
                if available_maps:
                    new_map_name = random.choice(available_maps)
                    self.map_path = os.path.join(maps_dir, new_map_name)
                    self.get_logger().info(f'Randomly selected new map: {new_map_name}')
            elif request.map_name:
                maps_dir = os.path.dirname(self.map_path)
                new_map_path = os.path.join(maps_dir, request.map_name)
                if os.path.exists(new_map_path):
                    self.map_path = new_map_path
                    self.get_logger().info(f'Switching to map: {request.map_name}')
                else:
                    self.get_logger().error(f'Requested map not found: {request.map_name}')
                    response.success = False
                    response.loaded_map_name = os.path.basename(self.map_path)
                    return response

            self.reset_requested = True
            response.success = True
            response.loaded_map_name = os.path.basename(self.map_path)
        return response

    def publish_sensor_data(self):
        msg = RobotSensors()
        surroundings = self.robot.check_surroundings()
        msg.up = surroundings['up']
        msg.down = surroundings['down']
        msg.left = surroundings['left']
        msg.right = surroundings['right']
        msg.up_left = surroundings['up_left']
        msg.up_right = surroundings['up_right']
        msg.down_left = surroundings['down_left']
        msg.down_right = surroundings['down_right']
        self.publisher_.publish(msg)

    def update(self):
        with self.state_lock:
            if self.reset_requested:
                self.get_logger().info(f'Resetting game with map: {os.path.basename(self.map_path)}')
                new_maze_config = load_from_csv(self.map_path)
                self.maze = Maze(new_maze_config, self.resolution)
                self.robot = Robot(self.maze)
                self.win = False
                self.win_handled = False
                self.reset_requested = False

        self.handle_input()
        if not self.win:
            self.maze.draw()

    def handle_map_request(self, request, response):
        occ_grid = self.maze.get_occupancy_grid()
        response.occupancy_grid_flattened = flatten(occ_grid)
        response.occupancy_grid_shape = len(occ_grid), len(occ_grid[0])
        return response

    def handle_move_cmd(self, request, response):
        if self.win:
            self.get_logger().warn('Game has already been won, ignoring move command.')
            response.success = False
            response.robot_pos = self.robot.pos
            # Target no longer exists, so use robot's final position.
            response.target_pos = self.robot.pos
            return response

        direction = request.direction.lower()
        target_pos = find(self.maze.get_occupancy_grid(), 't')
        response.success = self.robot.move(direction)
        if target_pos == self.robot.pos:
            self.win = True
        response.robot_pos = self.robot.pos
        response.target_pos = target_pos
        return response

    def handle_input(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_h:
                    direction = 'left'
                elif event.key == pygame.K_j:
                    direction = 'down'
                elif event.key == pygame.K_k:
                    direction = 'up'
                elif event.key == pygame.K_l:
                    direction = 'right'
                else:
                    return
                target_pos = find(self.maze.get_occupancy_grid(), 't')
                success = self.robot.move(direction)
                if target_pos == self.robot.pos:
                    self.win = True
                    self.maze.win()
        
    def run(self):
        pygame.init()
        self.running = True
        while self.running and rclpy.ok():
            self.update()
            pygame.display.flip()

            if self.win and not self.win_handled:
                self.win_handled = True
                self.maze.win()

        pygame.quit()
