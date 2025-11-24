import rclpy
from rclpy.node import Node
from pushcorp_msgs.msg import Value
from pushcorp_msgs.srv import SetControlMode, CommandValue
from std_srvs.srv import Trigger
import socket
import time 
import json

# For testing
import numpy as np


POSITION_ENDPOINT = "/afd/actualPosition"
FORCE_ENDPOINT = "/afd/actualForce"
COMMAND_FORCE_ENDPOINT = "/afd/commandForce"
COMMAND_POSITION_ENDPOINT = "/afd/commandPosition"
SET_CONTROL_MODE_ENDPOINT = "/afd/controlMode"
WEIGH_PAYLOAD_ENDPOINT = "/afd/weighPayload"
SUCCESS_VALUE = 'success'
STATUS_KEY = 'status'


def create_socket():
  """ Helper for creating a UDP socket
  """
  return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


class AFDDriver(Node):
    def __init__(self):
        super().__init__('afd_position_publisher')
        
        self.declare_parameter('ip')
        self.declare_parameter('port')
        self.declare_parameter('period')

        # Get the parameter values
        self.ip = self.get_parameter('ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.period = self.get_parameter('period').get_parameter_value().double_value

        # Create the ROS2 interfaces
        self.pos_pub = self.create_publisher(Value, 'afd/position', 10)
        self.force_pub = self.create_publisher(Value, 'afd/force', 10)
        self.weigh_payload_server = self.create_service(Trigger, 'weigh_payload', self.weigh_payload)
        self.set_control_mode_server = self.create_service(SetControlMode, 'set_control_mode', self.set_control_mode)
        self.command_force_server = self.create_service(CommandValue, 'command_force', self.command_force)
        self.command_position_server = self.create_service(CommandValue, 'command_position', self.command_position)

        # Create a timer for reading from the AFD
        self.timer = self.create_timer(self.period, self.publish_udp_data)

        # For testing
        self.start_time = 0
        self.end_time = 0
        self.udp_time = []
        self.median_udp_time = np.array(self.udp_time)

    def send(self,
             udp_socket: socket.socket,
             endpoint: str,
             buffer_size: int = 1024) -> dict:
      """ Sends data using a UDP socket
      """
      udp_socket.sendto(endpoint.encode(), (self.ip, self.port))
      response, _ = udp_socket.recvfrom(buffer_size)
      return json.loads(response.decode())

    def publish_udp_data(self): 
        def populate_value(udp_socket: socket.socket,
                           endpoint: str) -> Value:
            """ Helper function for evaluating a request into a ROS2 message
            """
            # Send the request
            response = self.send(udp_socket, endpoint)

            status = response[STATUS_KEY]
            if status != SUCCESS_VALUE:
                self.get_logger().info(f'Failed to read endpoint \'{endpoint}\': {status}')
                return None

            # Convert to message
            msg = Value()
            msg.value = response['data'][endpoint]

            msg.timestamp = self.get_clock().now().to_msg()

            return msg

        # Socket communication
        udp_socket = create_socket()
        # For testing
        udp_socket.settimeout(self.period * 1.3)
        try:
            # For testing
            self.start_time = time.time()
            pos_msg = populate_value(udp_socket, POSITION_ENDPOINT)
            if pos_msg is not None:
                self.pos_pub.publish(pos_msg)
                # For testing
                self.end_time = time.time()
                self.udp_time.append(self.end_time - self.start_time)
                udp_time_array = np.array(self.udp_time)
                self.median_udp_time = np.median(udp_time_array)
                print(f'Median udp time is {self.median_udp_time}')

            force_msg = populate_value(udp_socket, FORCE_ENDPOINT)
            if force_msg is not None:
                self.force_pub.publish(force_msg)

        except:
            self.get_logger().info('udp socket timed out')
        udp_socket.close()

    def weigh_payload(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        udp_socket = create_socket()
        response = self.send(udp_socket, WEIGH_PAYLOAD_ENDPOINT)
        res.success = response[STATUS_KEY] == SUCCESS_VALUE
        if not res.success:
            res.message = 'Failed to weigh payload'
        else:
            res.message = 'Weighed payload'
        
        udp_socket.close()
        return res

    def set_control_mode(self, req: SetControlMode.Request, res: SetControlMode.Response) -> SetControlMode.Response:
        udp_socket = create_socket()
        if not req.mode in (0, 1):
            res.success = False
            res.message = f'Mode must either be 0 (position) or 1 (force)'
            udp_socket.close()
            return res

        response = self.send(udp_socket, f'{SET_CONTROL_MODE_ENDPOINT}={req.mode}')
        res.success = response[STATUS_KEY] == SUCCESS_VALUE
        if not res.success:
            res.message = 'Failed to set command mode'
        else:
            mode = ""
            mode = "position" if (req.mode == 0) else "force"
            res.message = f'Set command mode to {mode}'
        
        udp_socket.close()
        return res

    def command_force(self, req: CommandValue.Request, res: CommandValue.Response) -> CommandValue.Response:
        udp_socket = create_socket()
        response = self.send(udp_socket, f'{COMMAND_FORCE_ENDPOINT}={req.value}')
        res.success = response[STATUS_KEY] == SUCCESS_VALUE
        if not res.success:
            res.message = 'Failed to command force'
        else: 
            res.message = f'Commanded force to {req.value}'
        udp_socket.close()
        return res

    def command_position(self, req: CommandValue.Request, res: CommandValue.Response) -> CommandValue.Response:
        udp_socket = create_socket()
        if req.value < 0 or req.value > 20:
            res.success = False
            res.message = f'Commanded position {req.value} must be in the range [0, 20]. Not setting position.'
            udp_socket.close()
            return res
        response = self.send(udp_socket, f'{COMMAND_POSITION_ENDPOINT}={req.value}')
        res.success = response[STATUS_KEY] == SUCCESS_VALUE
        if not res.success:
            res.message = 'Failed to command position'
        else:
            res.message = f'Commanded position to {req.value}'
        udp_socket.close()
        return res


def main(args=None):
    rclpy.init(args=args)
    afd_driver = AFDDriver()
    afd_driver.get_logger().info('Started AFD driver')
    rclpy.spin(afd_driver)
    afd_driver.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()


