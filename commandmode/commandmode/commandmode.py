"""
Control the status of the diffusion model inference and action deployment.

Using tty and sys to read keyboard inputs to trigger service calls to start
or stop the diffusion model inference and action deployment within the 
diffusion_policy node. Additionally, the command_mode is sent to the TODO: input node name
to trigger state updates.

PUBLISHERS:
  + /command_mode (String) - The command mode for the key pressed.
SERVICE CLIENTS:
  + /start_inference (Empty) - Enables inference with empty request
  + /start_action (Empty) - Enables action deployment with empty request
  + /stop_inference (Empty) - Disables inference with empty request
  + /stop_action (Empty) - Disables inference with empty request
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import String
from std_srvs.srv import Empty

import sys
import tty
import termios
from select import select


class CommandMode(Node):
    def __init__(self):
        super().__init__("commandmode")

        # create callback groups
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.waypoint_callback_group = MutuallyExclusiveCallbackGroup()

        # create timer
        self.timer = self.create_timer(1/30, self.timer_callback, callback_group=self.timer_callback_group)

        # publishers
        self.command_mode_pub = self.create_publisher(String, 'command_mode', 10)

        # service clients
        self.inference_start_client = self.create_client(Empty, 'start_inference')
        self.inference_start_client.wait_for_service(timeout_sec=2.0)

        self.inference_stop_client = self.create_client(Empty, 'stop_inference')
        self.inference_stop_client.wait_for_service(timeout_sec=2.0)

        self.action_start_client = self.create_client(Empty, 'start_action')
        self.action_start_client.wait_for_service(timeout_sec=2.0)

        self.action_stop_client = self.create_client(Empty, 'stop_action')
        self.action_stop_client.wait_for_service(timeout_sec=2.0)

        self.record_client = self.create_client(Empty, 'record')
        self.record_client.wait_for_service(timeout_sec=2.0)

        # create timer
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

        # keyboard hot keys
        self.get_logger().info("Press the letter 'b' to begin diffusion inference.\n")
        self.get_logger().info("Press the letter 'a' to begin diffusion actions.\n")
        self.get_logger().info("Press the letter 'p' to stop diffusion inference.\n")
        self.get_logger().info("Press the letter 's' to stop diffusion action.\n")
        self.get_logger().info("Press the letter 'x' to kill node.\n")
        self.settings = termios.tcgetattr(sys.stdin)
        self.timeout = 0.01

        self.listening = True
        self.command_mode = 'None'

    def getKey(self):
        """Read keyboard inputs from the terminal or extern window."""
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select([sys.stdin], [], [], self.timeout)
            if rlist:
                self.key = sys.stdin.read(1)
            else:
                self.key = None
        except Exception as e:
            self.get_logger().info(f'ERROR: {e}')
        
    def check_keys(self):
        """Check key press for hot keys and performs associated action."""
        if self.key == 'b':

            self.get_logger().info(f'Starting diffusion inference now.\r\n')
            self.command_mode = 'Begin'
            future = self.inference_start_client.call_async(Empty.Request())

        elif self.key == 'a':

            self.get_logger().info('Starting diffusion actions now.\r\n')
            self.command_mode = 'Action'
            future = self.action_start_client.call_async(Empty.Request())
            future = self.record_client.call_async(Empty.Request())
            
        elif self.key == 'p':

            self.get_logger().info(f'Stopping diffusion inference now.\r\n')
            self.command_mode = 'Pause'
            future = self.inference_stop_client.call_async(Empty.Request())

        elif self.key == 's':

            self.get_logger().info('Stopping diffusion actions now.\r\n')
            self.command_mode = 'Pause'
            future = self.action_stop_client.call_async(Empty.Request())

        elif self.key == 'x':

            self.get_logger().info(f'Terminating node now.\r\n')
            self.listening = False
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            raise 'Node Terminated.'

    def timer_callback(self):
        """Publish the command mode."""
        
        # listening for keys
        if self.listening:

            self.getKey()
            self.check_keys()

        # publish command mode
        self.command_mode_pub.publish(String(data=self.command_mode))


def main(args=None):
    rclpy.init(args=args)
    commandmode = CommandMode()
    rclpy.spin(commandmode)

if __name__ == '__main__':
    main()
