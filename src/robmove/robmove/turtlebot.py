import rclpy
from math import pi, atan2
from typing import List, Tuple, Self
from robmove.turtlesim_adapter import TurtlesimAdapter
from robmove.robot_controller import RobotController
from turtlesim.msg import Pose
from std_msgs.msg import String

# If independent, intended to be run with args set:
# ros2 run robmove turtlebot --ros-args -p independent:=True -p name:=turtlebot -p "home:=[
# 1.0, 3.0]" -p theta:=0.0 -p loopback:=True -p "goal:=[1.0, 3.0, 4.0, 3.0, 4.0, 6.0, 
# 1.0, 6.0]" -p update_freq:=10

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
    # update_freq: int, the frequency of state publishing, default is 10 Hz
    # dependent: bool, if True, the Turtlebot is dependent on the Turtlehub, default is False
    def __init__(self, name: str = "turtlebot", x: float = 1.0, y: float = 3.0, theta: float = 0.0,
                 max_lin_vel: float = 2.0, max_ang_vel: float = pi / 3,
                 smoothstop_linvel: float = 0.5, smoothstop_angvel: float = pi / 4, 
                 update_freq: int = 10, dependent: bool = False):
        # declare parameters for the Turtlebot
        super().__init__(name)
        self.declare_parameter('name', "turtlebot")
        self.declare_parameter('home', [1.0, 3.0])
        self.declare_parameter('theta', 0.0)
        self.declare_parameter('goals', [1.0, 3.0, 4.0, 3.0, 4.0, 6.0, 1.0, 6.0])
        self.declare_parameter('loopback', True)
        self.declare_parameter('update_freq', 10)
        self.declare_parameter('independent', True)
        independent = self.get_parameter('independent').value if not dependent else False
        self.independent = independent
        name = self.get_parameter('name').value if independent else name
        self.initialize(name) # initialize the TurtlesimAdapter using updated name
        # get the parameters for the Turtlebot
        home = self.get_parameter('home').value
        x, y = (home[0], home[1]) if independent else (x, y)
        theta = self.get_parameter('theta').value if independent else theta
        self.update_freq = self.get_parameter('update_freq').value if independent else update_freq
        tmpgoals: List[float] = self.get_parameter('goals').value
        self.loopback = self.get_parameter('loopback').value
        # initialize the robot controller
        self.bot = RobotController(name, 
                                   max_lin_vel = max_lin_vel, 
                                   max_ang_vel = max_ang_vel, 
                                   smoothstop_linvel = smoothstop_linvel, 
                                   smoothstop_angvel = smoothstop_angvel,
                                   drive_func = self.drive)
        self.paused_state = 'idle'
        # initialize turtlebot goals
        self.edit(tmpgoals)
        # spawns the turtlebot and subscribes to the cmd topic
        self.spawn_if_not_exists(x, y, theta)
        self.pose = Pose(x=x, y=y, theta=theta)
        self.get_logger().info(name + ' started')
        self.home(x, y, theta)
        self.bot.stop_callbacks['home'] = lambda: (self.home(x, y, theta) if self.loopback else None)
        self.subscription_cmd = self.create_subscription(String, name + '/cmd', self.cmd_callback, 10)
        # create a timer to publish the state of the Turtlebot
        duration = 1.0 / self.update_freq
        self.create_timer(duration, self.state_timer_callback)
        self.publisher_state = self.create_publisher(String, name + '/state', 10)
        self.countdown_timer = None
        self.repeat_timer = None

    # homes the Turtlebot to the initial position and orientation
    def home(self, x: float, y: float, theta: float):
        self.bot.set_init_pose(x, y, theta)
        self.bot.initialize_pose()
        self.teleport(x, y, theta, verbose=False)

    # callback function for state publishing timer
    def state_timer_callback(self):
        msg = String()
        state = self.bot.current
        x = self.pose.x
        y = self.pose.y
        theta = self.pose.theta
        v = self.pose.linear_velocity
        w = self.pose.angular_velocity
        msg.data = f'{state},{x},{y},{theta},{v},{w}'
        self.publisher_state.publish(msg)

    # edit the goals of the Turtlebot, the goals are specified as a list of floats
    def edit(self, data: List[float]):
        if len(data) % 2 != 0: # odd number of goals
            data = data[1:] # ignore the first element
        self.goals = self.param_to_goals(data)
        if self.loopback:
            self.set_goals_loopback(self.goals)
        else:
            self.set_goals(self.goals)
        self.get_logger().info(f'Edited goals: {self.goals}' + ' with loopback' if self.loopback else '')

    # convert a list of parameters to a list of goals
    def param_to_goals(self, param: List[float]) -> List[Tuple[float, float]]:
        return [(param[i], param[i+1]) for i in range(0, len(param), 2)]

    # start the Turtlebot after t seconds
    def startin(self, t: float):
        # checks if the countdown timer exists, if so, cancels it
        if self.countdown_timer:
            self.destroy_countdown_timer()
        if t <= 0: # no countdown
            self.start()
        else:
            self.countdown_timer = self.create_timer(t, self.countdown_timer_callback, autostart=True)
        
    # destroys the countdown timer
    def destroy_countdown_timer(self):
        self.countdown_timer.cancel()
        self.countdown_timer.destroy()
        self.countdown_timer = None

    # the callback function for the countdown timer
    def countdown_timer_callback(self):
        self.destroy_countdown_timer()
        self.start()

    # destroys the repeat timer
    def destroy_repeat_timer(self):
        self.repeat_timer.cancel()
        self.repeat_timer.destroy()
        self.repeat_timer = None

    # repeat the Turtlebot's goals after t seconds
    def repeat(self, t: float):
        # checks if the repeat timer exists, if so, cancels it
        if self.repeat_timer:
            self.destroy_repeat_timer()
        if t > 0:
            self.repeat_timer = self.create_timer(t, self.repeat_timer_callback, autostart=True)

    # callback function for the repeat timer
    def repeat_timer_callback(self):
        moving = self.bot.current != 'idle'
        if not moving:
            self.start()

    # callback function for cmd subscription
    def cmd_callback(self, msg: String):
        cmd = msg.data
        data = cmd.split(':') # for "cmd:pt1,pt2,..." format
        if len(data) > 1:
            cmd, data = data[0], data[1]
        # ignore some commands if the bot is not idle
        ignored_cmds = ['start', 'edit', 'editloop', 'repeat', 'home', 'startin']
        ignored: bool = self.bot.current != 'idle' and cmd in ignored_cmds
        if ignored:
            self.get_logger().info(f'Already moving, ignored command: {msg.data}')
        self.get_logger().info(f'Received command: {cmd}')
        if cmd == 'start':
            if not ignored:
                self.start()
        elif cmd == 'startin':
            if not ignored:
                t = float(data) if data else 0.0
                self.startin(t)
        elif cmd == 'repeat':
            if not ignored:
                t = float(data) if data else 0.0
                self.repeat(t)
        elif cmd == 'edit':
            if not ignored:
                self.loopback = False
                floatdata = [float(x) for x in data.split(',')]
                self.edit(floatdata)
        elif cmd == 'editloop':
            if not ignored:
                self.loopback = True
                floatdata = [float(x) for x in data.split(',')]
                self.edit(floatdata)
        elif cmd == 'home':
            if not ignored:
                floatdata = [float(x) for x in data.split(',')]
                self.home(data)
        elif cmd == 'stop':
            self.stop()
        elif cmd == 'pause':
            self.pause()
        elif cmd == 'resume':
            self.resume()
        elif cmd == 'kill':
            self.kill()
            if self.independent:
                raise KeyboardInterrupt
            else:
                self.destroy_node()

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
        super().pose_callback(msg)
        self.bot.update_pose(msg.x, msg.y, msg.theta)
        self.bot.run_state()

    # start the Turtlebot
    def start(self):
        self.bot.set_state('start')

    # stop the Turtlebot
    def stop(self):
        self.paused_state = 'idle'
        self.bot.set_state('stop')

    # pause the Turtlebot
    def pause(self):
        self.paused_state = self.bot.current
        self.bot.set_state('idle')

    # resume the Turtlebot
    def resume(self):
        self.bot.set_state(self.paused_state)
        self.paused_state = 'idle'

# main function for running the Turtlebot node on its own
def main(args=None):
    rclpy.init(args=args)
    try:
        bot = Turtlebot()
        rclpy.spin(bot)
        bot.destroy_node()
    except KeyboardInterrupt:
        print('Shutting down...')
    except Exception as e:
        readable_exception = e.__class__.__name__ + ': ' + str(e)
        print(readable_exception)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()