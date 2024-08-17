from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute

# TurtlesimAdapter class provides easy access to turtlesim functionalities
# such as spawn / teleport / move / pose
class TurtlesimAdapter(Node):

    # name: name of the turtlesim bot
    def __init__(self, name: str):
        super().__init__(name)
        self.name = name
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.teleport_client = self.create_client(TeleportAbsolute, name + '/teleport_absolute')
        self.publisher_cmd = self.create_publisher(Twist, name + '/cmd_vel', 10)
        self.subscription_pose = self.create_subscription(Pose, name + '/pose', self.pose_callback, 10)
        self.subscription_pose
    
    # callback function for pose subscription, empty by default, to be overridden
    def pose_callback(self, msg: Pose):
        pass
    
    # spawn a new turtlesim bot at (x, y, theta)
    def spawn(self, x: float, y: float, theta: float, verbose: bool = True):
        req = Spawn.Request()
        req.x = x
        req.y = y
        req.theta = theta
        req.name = self.name
        self.spawn_client.call_async(req)
        if verbose:
            self.get_logger().info('Spawned %s at (%f, %f, %f)' % (req.name, x, y, theta))

    # teleport the turtlesim bot to (x, y, theta)
    def teleport(self, x: float, y: float, theta: float, verbose: bool = True):
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        self.teleport_client.call_async(req)
        if verbose:
            self.get_logger().info('Teleported %s to (%f, %f, %f)' % (self.name, x, y, theta))

    # check if the turtlesim bot exists
    def exists(self) -> bool:
        topics = self.get_topic_names_and_types()
        topics = [t for t, _ in topics]
        # the /<name>/color_sensor topic is automatically created when the bot is spawned
        return f'/{self.name}/color_sensor' in topics
    
    # spawn the turtlesim bot if it does not exist, otherwise teleport it to (x, y, theta)
    def spawn_if_not_exists(self, x: float, y: float, theta: float, verbose: bool = True):
        if self.exists():
            self.teleport(x, y, theta, verbose=False)
            self.get_logger().info('Teleported existing %s to (%f, %f, %f)' % (self.name, x, y, theta))
        else:
            self.spawn(x, y, theta, verbose=verbose)

    # move the turtlesim bot with linear and angular velocity
    def move(self, linear: float, angular: float, verbose: bool = False):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_cmd.publish(msg)
        if verbose:
            self.get_logger().info('Move %s: linear=%f, angular=%f' % (self.name, linear, angular))