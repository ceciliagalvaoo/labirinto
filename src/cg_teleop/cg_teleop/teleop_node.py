import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd, Reset

import sys
import termios
import tty

msg = """
Reading from the keyboard and Publishing to /move_command!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For non-Vim users:
   w/s: up/down
   a/d: left/right

r : reset the current board
n : start a new random board

q/z : increase/decrease max speeds by 10%
anything else : stop

CTRL-C to quit
"""

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.move_cli = self.create_client(MoveCmd, '/move_command')
        self.reset_cli = self.create_client(Reset, '/reset')
        
        while not self.move_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Move service not available, waiting again...')
        while not self.reset_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Reset service not available, waiting again...')
            
        self.move_req = MoveCmd.Request()

    def send_move_request(self, direction):
        self.move_req.direction = direction
        self.future = self.move_cli.call_async(self.move_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_reset_request(self, is_random=False):
        reset_req = Reset.Request()
        reset_req.is_random = is_random
        self.future = self.reset_cli.call_async(reset_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopKeyboard()
    settings = termios.tcgetattr(sys.stdin)

    print(msg)
    while rclpy.ok():
        key = get_key(settings)
        direction = None
        if key in ['w', 'k', '\x1b[A']:  # w, k, up arrow
            direction = 'up'
        elif key in ['s', 'j', '\x1b[B']:  # s, j, down arrow
            direction = 'down'
        elif key in ['a', 'h', '\x1b[D']:  # a, h, left arrow
            direction = 'left'
        elif key in ['d', 'l', '\x1b[C']:  # d, l, right arrow
            direction = 'right'
        elif key == 'r':
            response = teleop_node.send_reset_request(is_random=False)
            if response.success:
                teleop_node.get_logger().info(f'Successfully reset board to map: {response.loaded_map_name}')
            else:
                teleop_node.get_logger().info('Failed to reset the board')
        elif key == 'n':
            response = teleop_node.send_reset_request(is_random=True)
            if response.success:
                teleop_node.get_logger().info(f'Successfully loaded new random map: {response.loaded_map_name}')
            else:
                teleop_node.get_logger().info('Failed to load new random map')
        elif key == '\x03':  # ctrl-c
            break
        
        if direction:
            response = teleop_node.send_move_request(direction)
            if response.success:
                teleop_node.get_logger().info(f'Successfully moved {direction}')
            else:
                teleop_node.get_logger().info(f'Failed to move {direction}')

    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
