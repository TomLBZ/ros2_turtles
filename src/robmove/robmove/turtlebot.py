from math import pi, atan2
from typing import List, Tuple
from robmove.turtlesim_adapter import TurtlesimAdapter
from robmove.robot_controller import RobotController
from turtlesim.msg import Pose
from std_msgs.msg import String

# Turtlebot class extends TurtlesimAdapter, it provides a RobotController based state machine
# for controlling a robot in turtlesim. It provides methods for setting goals and moving the robot
# to those goals. The goals are specified as a list of (x, y) tuples. The robot moves to each goal
# in sequence, turning towards the goal first and then moving forward. The robot can be started, stopped,
# paused and resumed.
class Turtlebot(TurtlesimAdapter):

    # name: string, the name of the Turtlebot
    # x: float, the initial x position of the Turtlebot
    # y: float, the initial y position of the Turtlebot
    # theta: float, the initial orientation of the Turtlebot in radians, default is 0.0
    # max_lin_vel: float, the maximum linear velocity of the Turtlebot, default is 3.0
    # max_ang_vel: float, the maximum angular velocity of the Turtlebot, default is pi / 3
    # smoothstop_linvel: float, the linear velocity below which smoothstop kicks in, default is 0.5
    # smoothstop_angvel: float, the angular velocity below which smoothstop kicks in, default is pi / 4
    def __init__(self, name: str, x: float, y: float, theta: float = 0.0,
                 max_lin_vel: float = 3.0, max_ang_vel: float = pi / 3,
                 smoothstop_linvel: float = 0.5, smoothstop_angvel: float = pi / 4):
        self.name = name
        self.bot = RobotController(name, 
                                   max_lin_vel = max_lin_vel, 
                                   max_ang_vel = max_ang_vel, 
                                   smoothstop_linvel = smoothstop_linvel, 
                                   smoothstop_angvel = smoothstop_angvel,
                                   drive_func = self.drive)
        self.bot.set_init_pose(x, y, theta)
        self.bot.initialize_pose()
        self.paused_state = 'idle'
        super().__init__(name)
        self.spawn_if_not_exists(x, y, theta)
        self.get_logger().info(name + ' started')
        self.subscription_cmd = self.create_subscription(String, name + '/cmd', self.cmd_callback, 10)

    # callback function for cmd subscription
    def cmd_callback(self, msg: String):
        if self.bot.current != 'idle':
            return # ignore commands if the bot is not idle
        cmd = msg.data
        self.get_logger().info(f'[{self.name}] Received command: {cmd}')
        if cmd == 'start':
            self.start()
        elif cmd == 'stop':
            self.stop()
        elif cmd == 'pause':
            self.pause()
        elif cmd == 'resume':
            self.resume()

    # get the current state of the Turtlebot
    def get_state(self) -> str:
        return self.bot.current

    # set the goals for the Turtlebot, the goals are specified as a list of (x, y) tuples
    # after reaching all goals, the Turtlebot returns to the initial position and orientation
    def set_goals_loopback(self, goals: List[Tuple[float, float]]):
        self.bot.reset_states()
        goals.append((self.bot.init_x, self.bot.init_y))
        if len(goals) == 0:
            self.bot.add_state('start', lambda: self.bot.set_state('idle'))
            return
        for i, goal in enumerate(goals):
            self.goal_to_states(goal, i)
        dtheta = self.bot.ang_dist(self.bot.theta, self.bot.init_theta)
        self.bot.add_state(f'turn{len(goals)}', lambda: self.bot.turn(dtheta, 'stop'))
        self.bot.update_pose(self.bot.init_x, self.bot.init_y, self.bot.init_theta)
        self.bot.add_state('start', lambda: self.bot.set_state('turn0'))

    # set the goals for the Turtlebot, the goals are specified as a list of (x, y) tuples
    # after reaching all goals, the Turtlebot stops with current orientation at the last goal
    def set_goals(self, goals: List[Tuple[float, float]]):
        self.bot.reset_states()
        if len(goals) == 0:
            self.bot.add_state('start', lambda: self.bot.set_state('idle'))
            return
        for i, goal in enumerate(goals):
            self.goal_to_states(goal, i)
        self.bot.add_state(f'turn{len(goals)}', lambda: self.bot.set_state('stop'))
        self.bot.add_state('start', lambda: self.bot.set_state('turn0'))

    # convert a goal to a sequence of states for turning and moving towards the goal
    def goal_to_states(self, goal: Tuple[float, float], index: int):
        gx, gy = goal
        dx = gx - self.bot.x
        dy = gy - self.bot.y
        gtheta = atan2(dy, dx)
        dtheta = self.bot.ang_dist(self.bot.theta, gtheta)
        dist = (dx**2 + dy**2)**0.5
        self.bot.add_state(f'turn{index}', lambda: self.bot.turn(dtheta, f'fwd{index}'))
        self.bot.add_state(f'fwd{index}', lambda: self.bot.forward(dist, f'turn{index+1}'))
        self.bot.update_pose(gx, gy, gtheta)

    # move the Turtlebot with linear and angular velocity
    def drive(self, linear: float, angular: float):
        self.move(linear, angular, verbose=False)

    # callback function for pose subscription, override the default pose_callback
    def pose_callback(self, msg: Pose):
        self.bot.update_pose(msg.x, msg.y, msg.theta)
        self.bot.run_state()

    # start the Turtlebot
    def start(self):
        self.bot.set_state('start')

    # stop the Turtlebot
    def stop(self):
        self.paused_state = 'idle'
        self.bot.set_state('stop')
        self.teleport(self.bot.init_x, self.bot.init_y, self.bot.init_theta, verbose=False)

    # pause the Turtlebot
    def pause(self):
        self.paused_state = self.bot.current
        self.bot.set_state('idle')

    # resume the Turtlebot
    def resume(self):
        self.bot.set_state(self.paused_state)
        self.paused_state = 'idle'
