"""Provided class: TurtlesimAdapter\n
TurtlesimAdapter can be inherited to provide easy access to turtlesim functionalities
such as spawn / teleport / move / kill / pose.
"""

from rclpy.node import Node, Client, Publisher, Subscription
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute, Kill
from typing import Self, List, Tuple

class TurtlesimAdapter(Node):
    """
    `TurtlesimAdapter` class provides easy access to `turtlesim` functionalities
    such as:
    - spawn
    - teleport
    - move
    - kill
    - pose\n
    It contains necessary publishers, subscribers, and clients 
    that work with the ros2 `turtlesim` package.
    """

    def __init__(self, name: str) -> Self:
        """Instantiate the `TurtlesimAdapter` with the given `name`"""
        super().__init__(name)
    
    def initialize(self, name: str) -> None:
        """Initialize the `TurtlesimAdapter` with the given `name`, MUST be called after the constructor"""
        self.name: str = name
        self.spawn_client: Client = self.create_client(Spawn, 'spawn')
        self.kill_client: Client = self.create_client(Kill, 'kill')
        self.teleport_client: Client = self.create_client(TeleportAbsolute, name + '/teleport_absolute')
        self.publisher_cmd: Publisher = self.create_publisher(Twist, name + '/cmd_vel', 10)
        self.subscription_pose: Subscription = self.create_subscription(Pose, name + '/pose', self.pose_callback, 10)
        self.pose: Pose = Pose() # current pose of the bot
        self.subscription_pose # prevent unused variable warning

    def pose_callback(self, msg: Pose) -> None:
        """Callback function for pose subscriber"""
        self.pose = msg
    
    def spawn(self, x: float, y: float, theta: float, verbose: bool = True) -> None:
        """Spawn a new turtlesim bot at (`x`, `y`, `theta`)"""
        req: Spawn.Request = Spawn.Request()
        req.x = x
        req.y = y
        req.theta = theta
        req.name = self.name
        self.spawn_client.call_async(req)
        if verbose:
            self.get_logger().info('Spawned %s at (%f, %f, %f)' % (req.name, x, y, theta))

    def kill(self, verbose: bool = True) -> None:
        """Kill the turtlesim bot"""
        req: Kill.Request = Kill.Request()
        req.name = self.name
        self.kill_client.call_async(req)
        if verbose:
            self.get_logger().info('Killed %s' % self.name)

    def teleport(self, x: float, y: float, theta: float, verbose: bool = True) -> None:
        """Teleport the turtlesim bot to (`x`, `y`, `theta`)"""
        req: TeleportAbsolute.Request = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        self.teleport_client.call_async(req)
        self.pose.x = x
        self.pose.y = y
        self.pose.theta = theta
        self.pose_callback(self.pose)
        if verbose:
            self.get_logger().info('Teleported %s to (%f, %f, %f)' % (self.name, x, y, theta))

    def exists(self) -> bool:
        """Check if the turtlesim bot exists"""
        topics: List[Tuple[str, List[str]]] = self.get_topic_names_and_types()
        topics: List[str] = [t for t, _ in topics]
        # the /<name>/color_sensor topic is automatically created when the bot is spawned
        return f'/{self.name}/color_sensor' in topics
    
    def spawn_if_not_exists(self, x: float, y: float, theta: float, verbose: bool = True) -> None:
        """Spawn the turtlesim bot if it does not exist, otherwise teleport it to (`x`, `y`, `theta`)"""
        if self.exists():
            self.teleport(x, y, theta, verbose=False)
            self.get_logger().info('Teleported existing %s to (%f, %f, %f)' % (self.name, x, y, theta))
        else:
            self.spawn(x, y, theta, verbose=verbose)

    def move(self, linear: float, angular: float, verbose: bool = False) -> None:
        """Move the turtlesim bot with `linear` and `angular` velocities"""
        msg: Twist = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_cmd.publish(msg)
        if verbose:
            self.get_logger().info('Move %s: linear=%f, angular=%f' % (self.name, linear, angular))