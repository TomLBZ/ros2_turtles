import websockets.connection
import websockets.exceptions
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import websockets
import asyncio

# ROS parameters are given by:
# ros2 run robmove ros2socket --ros-args --params-file config/ros2socket.yaml

# Ros2Socket is a ROS2 node that:
# 1. Contains a WebSocket server that listens to a given address and port
# 2. Publishes the state of the WebSocket connection periodically
# 3. Subscribes to a given topic and sends the message to the WebSocket clients connected, if any
# 4. Publishes the message received from the WebSocket server to a given topic
# 5. Do not block the main thread
class Ros2Socket(Node):
    def __init__(self):
        self.name = 'ros2socket'
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
        self.pub = self.create_publisher(String, self.output_topic, 10)
        self.state_pub = self.create_publisher(Bool, self.state_topic, 10)
        self.sub = self.create_subscription(String, self.input_topic, self.send_callback, 10)
        # sets the period for the state publisher
        self.period: float = 1.0 / self.state_publish_rate_hz
        self.timer = self.create_timer(self.period, self.timer_callback)
        # variables for managing the websocket connection
        self.state: bool = False # False: disconnected, True: connected
        self.connected_clients = set()

    # periodically publish the state of the websocket connection
    def timer_callback(self):
        self.state = len(self.connected_clients) > 0
        msg = Bool()
        msg.data = self.state
        self.state_pub.publish(msg)

    # after subscriber received a message, send it to the websocket clients
    def send_callback(self, msg):
        if self.connected_clients:
            message: str = msg.data
            if message is not None:
                asyncio.create_task(self.send_to_clients(message))

    # after receiving data from websocket, publish it to the output topic
    async def websocket_handler(self, websocket, path):
        self.connected_clients.add(websocket)
        self.get_logger().info('Client connected')
        try:
            async for message in websocket:
                msg = String()
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

    # send the message to the websocket clients
    async def send_to_clients(self, message : str):
        if self.connected_clients:
            try:
                await asyncio.gather(*[websocket.send(message) for websocket in self.connected_clients])
            except Exception as e:
                readable_exception = e.__class__.__name__ + ': ' + str(e)
                self.get_logger().error(readable_exception)

def stop_all_tasks(loop: asyncio.AbstractEventLoop):
    tasks = asyncio.all_tasks(loop)
    for task in tasks:
        task.cancel()
    loop.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))

# manually spin the node. blocking, so must be run as an async task
async def manual_spin(node: Ros2Socket, sleep_ms: float = 0.0, loop: asyncio.AbstractEventLoop = None):
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=sleep_ms)
            if sleep_ms >= 0:
                await asyncio.sleep(sleep_ms)
    except KeyboardInterrupt:
        if loop is not None:
            stop_all_tasks(loop)
        raise KeyboardInterrupt # propagate after stopping all tasks at hand
    except Exception as e:
        readable_exception = e.__class__.__name__ + ': ' + str(e)
        node.get_logger().error(readable_exception)

# main function
def main():
    rclpy.init()
    r2s = Ros2Socket()
    serve = websockets.serve(r2s.websocket_handler, r2s.address, r2s.port)
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(serve)
        loop.create_task(manual_spin(r2s, loop=loop))
        loop.run_forever()
    except KeyboardInterrupt:
        # gracefully stop all tasks and shutdown
        stop_all_tasks(loop)
        if rclpy.ok():
            rclpy.shutdown()
        loop.stop()
        print(' ros2socket gracefully stopped')
    except Exception as e:
        readable_exception = e.__class__.__name__ + ': ' + str(e)
        r2s.get_logger().error(readable_exception)
    finally:
        loop.close()

if __name__ == '__main__':
    main()