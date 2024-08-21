"""This file can be run on its own to start a `ros2socket` node using the 
`ros2 run` command. It also provides a `Ros2Socket` class that can be instantiated 
and run in a separate script.
"""

import websockets
import asyncio
import rclpy
import websockets.connection
import websockets.exceptions
from asyncio import AbstractEventLoop
from rclpy.node import Node, Publisher, Subscription, Timer
from std_msgs.msg import String, Bool
from typing import Self, Set

class Ros2Socket(Node):
    """
    `Ros2Socket` is a ROS2 node that:
    - Contains a WebSocket server that listens to a given address and port
    - Publishes the state of the WebSocket connection periodically
    - Subscribes to a given topic and sends the message to the WebSocket clients connected, if any
    - Publishes the message received from the WebSocket server to a given topic
    """

    def __init__(self) -> Self:
        """
        Instantiates a `Ros2Socket` object. Parameters can be specified either by
        passing them directly to `ros2 run` or by specifying them in a YAML file.
        Parameters are:
        - `address`: the address to listen to
        - `port`: the port to listen to
        - `input_topic`: the topic to subscribe to for messages to send to the WebSocket clients
        - `output_topic`: the topic to publish messages received from the WebSocket clients
        - `state_topic`: the topic to publish the state of the WebSocket connection
        - `state_publish_rate_hz`: the rate at which to publish the state of the WebSocket connection
        """
        self.name: str = 'ros2socket'
        super().__init__(self.name)
        # declares the parameters
        self.declare_parameters(namespace='', parameters=[
            ('address', 'localhost'),
            ('port', 8765),
            ('input_topic', '/ros2socket/tosocket'),
            ('output_topic', '/ros2socket/fromsocket'),
            ('state_topic', '/ros2socket/state'),
            ('state_publish_rate_hz', 10)
        ])
        # gets the parameters and assigns them to the corresponding variables
        self.address: str = self.get_parameter('address').value
        self.port: int = self.get_parameter('port').value
        self.input_topic: str = self.get_parameter('input_topic').value
        self.output_topic: str = self.get_parameter('output_topic').value
        self.state_topic: str = self.get_parameter('state_topic').value
        self.state_publish_rate_hz: int = self.get_parameter('state_publish_rate_hz').value
        # creates the publishers and the subscriber
        self.pub: Publisher = self.create_publisher(String, self.output_topic, 10)
        self.state_pub: Publisher = self.create_publisher(Bool, self.state_topic, 10)
        self.sub: Subscription = self.create_subscription(String, self.input_topic, self.send_callback, 10)
        # sets the period for the state publisher
        self.period: float = 1.0 / self.state_publish_rate_hz
        self.timer: Timer = self.create_timer(self.period, self.timer_callback)
        # variables for managing the websocket connection
        self.state: bool = False # False: disconnected, True: connected
        self.connected_clients: Set = set()

    def timer_callback(self) -> None:
        """Periodically publish the state of the WebSocket connection on timer callback"""
        self.state: bool = len(self.connected_clients) > 0
        msg: Bool = Bool()
        msg.data = self.state
        self.state_pub.publish(msg)

    def send_callback(self, msg: String) -> None:
        """Send the `msg` received from the input topic to the WebSocket clients asynchronously"""
        if self.connected_clients:
            message: str = msg.data
            if message is not None:
                asyncio.create_task(self.send_to_clients(message))

    async def websocket_handler(self, websocket) -> None:
        """
        Handle the given `websocket` connection:
        - Add the `websocket` to the connected clients
        - After receiving data from websocket, publish it to the output topic
        - Remove the `websocket` from the connected clients if it is disconnected or an exception is thrown
        """
        self.connected_clients.add(websocket)
        self.get_logger().info('Client connected')
        try:
            async for message in websocket:
                msg: String = String()
                msg.data = message
                self.pub.publish(msg)
        except websockets.exceptions.ConnectionClosedError:
            self.get_logger().info('Connection closed by client')
        except asyncio.CancelledError:
            await websocket.close()
        except Exception as e:
            readable_exception = e.__class__.__name__ + ': ' + str(e)
            self.get_logger().error(readable_exception)
        finally:
            await websocket.close()
            # remove the websocket from the connected clients if it is disconnected or an exception is thrown
            self.connected_clients.remove(websocket)

    async def send_to_clients(self, message : str) -> None:
        """Send the `message` to all the connected clients asynchronously"""
        if self.connected_clients:
            try:
                await asyncio.gather(*[websocket.send(message) for websocket in self.connected_clients])
            except Exception as e:
                readable_exception: str = e.__class__.__name__ + ': ' + str(e)
                self.get_logger().error(readable_exception)

def stop_all_tasks(loop: AbstractEventLoop) -> None:
    """Stop all tasks in the given `loop`"""
    tasks = asyncio.all_tasks(loop)
    for task in tasks:
        if task.done():
            continue
        if task.cancelling():
            while task.cancelling() > 1:
                task.uncancel()
            continue
        if task.cancelled():
            continue
        task.cancel()
    loop.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))

async def manual_spin(node: Ros2Socket, sleep_ms: float = 0.0, loop: AbstractEventLoop = None) -> None:
    """Manually spin the given `node` with a sleep time of `sleep_ms` and `loop`"""
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=sleep_ms)
            if sleep_ms >= 0:
                await asyncio.sleep(sleep_ms)
    except KeyboardInterrupt:
        if loop is not None:
            stop_all_tasks(loop)
    except Exception as e:
        readable_exception: str = e.__class__.__name__ + ': ' + str(e)
        node.get_logger().error('[r2s:manual_spin] ' + readable_exception)

def main() -> None:
    """Main function to run the `ros2socket` node"""
    rclpy.init()
    r2s: Ros2Socket = Ros2Socket()
    serve = websockets.serve(r2s.websocket_handler, r2s.address, r2s.port)
    loop: AbstractEventLoop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(serve)
        loop.create_task(manual_spin(r2s, loop=loop))
        loop.run_forever()
    except KeyboardInterrupt:
        # gracefully stop all tasks and shutdown
        if loop is not None:
            stop_all_tasks(loop)
        if rclpy.ok():
            rclpy.shutdown()
        loop.stop()
        print(' ros2socket gracefully stopped')
    except Exception as e:
        readable_exception = e.__class__.__name__ + ': ' + str(e)
        r2s.get_logger().error('[r2s:main] ' + readable_exception)
    finally:
        loop.close()

if __name__ == '__main__':
    main()