"""This file can be run on its own to start a `Turtlebot` node using the 
`ros2 run` command. It also provides a `Turtlebot` class that can be instantiated 
and used by other `Nodes`.\n
Note that the `Turtlebot` is a robot that moves in and depends on the `turtlesim` package.\n
Also note that `Turtlebot`s must be spinned separately if used in another `Node`
as they are `Node`s themselves. If not, they will block the main `Node` from spinning.
"""

import rclpy
from rclpy.node import Publisher, Subscription, Timer
from math import pi, atan2
from typing import List, Tuple, Self
from robmove.turtlesim_adapter import TurtlesimAdapter
from robmove.robot_controller import RobotController
from turtlesim.msg import Pose
from std_msgs.msg import String

class Turtlebot(TurtlesimAdapter):
    """
    `Turtlebot` class extends `TurtlesimAdapter`, it provides a `RobotController` 
    based `StateMachine` for controlling a robot in turtlesim. It provides methods 
    for setting goals and moving the robot. It also allows the robot to be started, 
    stopped, paused and resumed. The robot can also be set to repeat its goals 
    after a set amount of time, or start after a countdown.\n
    Note that if `Turtlebot` is instantiated in another script, its `dependent` 
    parameter MUST be set to `True`, as only when `dependent=True` will other 
    parameters in the constructor take precedence over the parameters in the 
    parameter server or in a YAML file passed in. Otherwise, the `Turtlebot` may
    fail if the parameters are not set in the parameter server or in a YAML file.\n
    If the `Turtlebot` is run on its own, its `dependent` parameter can be left as `False`.
    """

    def __init__(self, name: str = "turtlebot", x: float = 1.0, y: float = 3.0, theta: float = 0.0,
                 max_lin_vel: float = 2.0, max_ang_vel: float = pi / 3,
                 smoothstop_linvel: float = 0.5, smoothstop_angvel: float = pi / 4, 
                 update_freq: float = 10, dependent: bool = False) -> Self:
        """
        Instantiate a `Turtlebot` with the given parameters:
        - `name`: the name of the `Turtlebot`
        - `x`: the initial x position
        - `y`: the initial y position
        - `theta`: the initial orientation in radians
        - `max_lin_vel`: the maximum linear velocity
        - `max_ang_vel`: the maximum angular velocity
        - `smoothstop_linvel`: below which linear velocity is smoothed
        - `smoothstop_angvel`: below which angular velocity is smoothed
        - `update_freq`: the frequency of state publishing
        - `dependent`: Set to `True` if the `Turtlebot` is run in another script, otherwise `False`"""
        # declare parameters for the Turtlebot
        super().__init__(name)
        self.declare_parameter('name', "turtlebot")
        self.declare_parameter('home', [1.0, 3.0])
        self.declare_parameter('theta', 0.0)
        self.declare_parameter('goals', [1.0, 3.0, 4.0, 3.0, 4.0, 6.0, 1.0, 6.0])
        self.declare_parameter('loopback', True)
        self.declare_parameter('update_freq', 10)
        self.declare_parameter('independent', True)
        # check if the Turtlebot is dependent on another script
        independent: bool = self.get_parameter('independent').value if not dependent else False
        self.independent: bool = independent
        # get the name of the Turtlebot
        name: str = self.get_parameter('name').value if independent else name
        self.initialize(name) # initialize the TurtlesimAdapter using updated name
        # get the other parameters for the Turtlebot
        home: List[float] = self.get_parameter('home').value
        x, y = (home[0], home[1]) if independent else (x, y)
        theta: float = self.get_parameter('theta').value if independent else theta
        self.update_freq: float = self.get_parameter('update_freq').value if independent else update_freq
        tmpgoals: List[float] = self.get_parameter('goals').value
        self.loopback: bool = self.get_parameter('loopback').value
        # initialize the robot controller
        self.bot: RobotController = RobotController(name, 
                                                    max_lin_vel = max_lin_vel, 
                                                    max_ang_vel = max_ang_vel, 
                                                    smoothstop_linvel = smoothstop_linvel, 
                                                    smoothstop_angvel = smoothstop_angvel,
                                                    drive_func = self.drive)
        # initialize the paused state
        self.paused_state: str = 'idle'
        # initialize turtlebot goals
        self.edit(tmpgoals)
        # spawns the turtlebot and subscribes to the cmd topic
        self.spawn_if_not_exists(x, y, theta)
        self.pose: Pose = Pose(x=x, y=y, theta=theta)
        self.get_logger().info(name + ' started')
        self.home(x, y, theta) # homes the Turtlebot
        # set the home callback to be either the home function or an empty lambda
        # depending on the loopback parameter
        self.bot.stop_callbacks['home'] = lambda: (self.home(x, y, theta) if self.loopback else None)
        # create a subscription to the cmd topic, where commands are received
        self.subscription_cmd: Subscription = self.create_subscription(String, name + '/cmd', self.cmd_callback, 10)
        # create a timer to publish the state of the Turtlebot
        duration: float = 1.0 / self.update_freq
        self.create_timer(duration, self.state_timer_callback)
        self.publisher_state: Publisher = self.create_publisher(String, name + '/state', 10)
        # initialize the countdown timer and repeat timer to None
        self.countdown_timer: Timer = None
        self.repeat_timer: Timer = None

    ####### Helper Methods #######

    def param_to_goals(self, param: List[float]) -> List[Tuple[float, float]]:
        """Convert a `list` of `float` `param` to a `list` of `(x, y)` tuples"""
        return [(param[i], param[i+1]) for i in range(0, len(param), 2)]

    def destroy_countdown_timer(self) -> None:
        """Destroys the countdown timer"""
        self.countdown_timer.cancel()
        self.countdown_timer.destroy()
        self.countdown_timer = None

    def destroy_repeat_timer(self) -> None:
        """Destroys the repeat timer"""
        self.repeat_timer.cancel()
        self.repeat_timer.destroy()
        self.repeat_timer = None

    def goal_to_states(self, goal: Tuple[float, float], index: int) -> None:
        """
        Convert a `goal` with the format (`x`, `y`) to a sequence of states for 
        turning and moving towards the goal.\n
        Note that the `index` is used to create unique state names for each goal,
        and the `Turtlebot` will transition to the state `turnk` after reaching 
        the goal at index `k-1`. Therefore a state `turnk` should be added to the
        `bot` after calling this method.\n
        Also note that the `Turtlebot` must be homed after adding all the goals.
        """
        gx, gy = goal
        dx: float = gx - self.bot.x
        dy: float = gy - self.bot.y
        goal_theta: float = atan2(dy, dx)
        dtheta: float = self.bot.ang_dist(self.bot.theta, goal_theta)
        dist: float = (dx**2 + dy**2)**0.5
        # add states: turn towards goal first then move forward
        self.bot.add_state(f'turn{index}', lambda: self.bot.turn(dtheta, f'fwd{index}'))
        self.bot.add_state(f'fwd{index}', lambda: self.bot.forward(dist, f'turn{index+1}'))
        # update the position and orientation for the bot to prepare for the next call to this method.
        # this is necessary to ensure that the bot moves towards the next goal from the last goal
        # and not from the initial position.
        # Note that after adding all the goals, the bot must be homed.
        self.bot.update_pose(gx, gy, goal_theta)

    def set_goals(self, goals: List[Tuple[float, float]]) -> None:
        """
        Set the goals for the `Turtlebot` with the given `goals` in the format 
        (`x`, `y`). The `Turtlebot` will stop with the current orientation at the last goal
        """
        self.bot.reset_states() # reset the states of the Turtlebot to only 'idle' and 'stop'
        if len(goals) == 0:
            # if no goals, set the entry state to idle
            self.bot.add_state('start', lambda: self.bot.set_state('idle'))
            return
        for i, goal in enumerate(goals):
            self.goal_to_states(goal, i) # convert each goal to states and add them to the bot
        # add a state for stopping at the last goal
        self.bot.add_state(f'turn{len(goals)}', lambda: self.bot.set_state('stop'))
        # add an entry state for starting the bot
        self.bot.add_state('start', lambda: self.bot.set_state('turn0'))

    def set_goals_loopback(self, goals: List[Tuple[float, float]]) -> None:
        """
        Set the goals for the `Turtlebot` with the given `goals` in the format 
        (`x`, `y`), and ensure that it returns to the initial position and 
        orientation after reaching all goals
        """
        self.bot.reset_states() # reset the states of the Turtlebot to only 'idle' and 'stop'
        goals.append((self.bot.init_x, self.bot.init_y)) # add the initial position as the last goal
        if len(goals) == 0:
            # if no goals, set the entry state to idle
            self.bot.add_state('start', lambda: self.bot.set_state('idle'))
            return
        for i, goal in enumerate(goals):
            self.goal_to_states(goal, i) # convert each goal to states and add them to the bot
        # compute and add the state for turning back to the initial orientation
        dtheta: float = self.bot.ang_dist(self.bot.theta, self.bot.init_theta)
        # add a state for stopping after turning back to the initial orientation
        self.bot.add_state(f'turn{len(goals)}', lambda: self.bot.turn(dtheta, 'stop'))
        # update the pose to allign theta with the initial theta
        self.bot.update_pose(self.bot.init_x, self.bot.init_y, self.bot.init_theta)
        # add an entry state for starting the bot
        self.bot.add_state('start', lambda: self.bot.set_state('turn0'))

    def drive(self, linear: float, angular: float):
        """Move the `Turtlebot` with the given `linear` and `angular` velocities"""
        self.move(linear, angular, verbose=False)

    ####### Callbacks #######

    def state_timer_callback(self) -> None:
        """Publish the current status of the `Turtlebot` as `state,x,y,theta,v,w`"""
        msg: String = String()
        state: str = self.bot.current
        x: float = self.pose.x
        y: float = self.pose.y
        theta: float = self.pose.theta
        v: float = self.pose.linear_velocity
        w: float = self.pose.angular_velocity
        msg.data = f'{state},{x},{y},{theta},{v},{w}'
        self.publisher_state.publish(msg)

    def countdown_timer_callback(self) -> None:
        """Callback function for the countdown timer"""
        self.destroy_countdown_timer()
        self.start()

    def repeat_timer_callback(self) -> None:
        """Callback function for the repeat timer"""
        moving: bool = self.bot.current != 'idle'
        if not moving:
            self.start()

    def pose_callback(self, msg: Pose) -> None:
        """
        Callback function for the pose subscription, overrides the default 
        `pose_callback` to update the pose of the `Turtlebot` and run its state
        """
        super().pose_callback(msg)
        self.bot.update_pose(msg.x, msg.y, msg.theta)
        self.bot.run_state()

    def cmd_callback(self, msg: String) -> None:
        """
        Callback function for the cmd subscription. Supported commands are:
        - `start`: start the `Turtlebot`
        - `startin:t`: start the `Turtlebot` after `t` seconds
        - `repeat:t`: repeat the goals of the `Turtlebot` after `t` seconds
        - `edit:x1,y1,x2,y2,...`: edit the goals of the `Turtlebot`
        - `editloop:x1,y1,x2,y2,...`: edit the goals of the `Turtlebot` such that 
            it loops back to the initial position
        - `home:x,y,theta`: home the `Turtlebot` to (`x`, `y`) with orientation `theta`
        - `stop`: stop the `Turtlebot`
        - `pause`: pause the `Turtlebot`
        - `resume`: resume the `Turtlebot`
        - `kill`: kill the `Turtlebot`
        """
        cmd: str = msg.data
        data: List[str] = cmd.split(':') # for "cmd:pt1,pt2,..." format
        if len(data) > 1:
            cmd, data = data[0], data[1]
        # ignore some commands if the bot is not idle
        ignored_cmds: List[str] = ['start', 'edit', 'editloop', 'repeat', 'home', 'startin']
        ignored: bool = (self.bot.current != 'idle') and (cmd in ignored_cmds)
        if ignored:
            self.get_logger().info(f'Already moving, ignored command: {msg.data}')
        self.get_logger().info(f'Received command: {cmd}')
        if cmd == 'start':
            if not ignored:
                self.start()
        elif cmd == 'startin':
            if not ignored:
                t: float = float(data) if data else 0.0
                self.startin(t)
        elif cmd == 'repeat':
            if not ignored:
                t: float = float(data) if data else 0.0
                self.repeat(t)
        elif cmd == 'home':
            if not ignored:
                floatdata: List[float] = [float(x) for x in data.split(',')]
                self.home(data)
        elif cmd == 'edit':
            if not ignored:
                self.loopback = False
                floatdata: List[float] = [float(x) for x in data.split(',')]
                self.edit(floatdata)
        elif cmd == 'editloop':
            if not ignored:
                self.loopback = True
                floatdata: List[float] = [float(x) for x in data.split(',')]
                self.edit(floatdata)
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

    ####### Public Methods #######

    def get_state(self) -> str:
        """Get the current state of the `Turtlebot`"""
        return self.bot.current

    def start(self) -> None:
        """Start the `Turtlebot`"""
        self.bot.set_state('start')

    def startin(self, t: float) -> None:
        """Start the `Turtlebot` after `t` seconds. If `t <= 0`, the `Turtlebot` starts immediately"""
        # checks if the countdown timer exists, if so, cancels it
        if self.countdown_timer:
            self.destroy_countdown_timer()
        if t <= 0: # no countdown, start immediately
            self.start()
        else: # start the countdown timer
            self.countdown_timer = self.create_timer(t, self.countdown_timer_callback, autostart=True)
        
    def repeat(self, t: float) -> None:
        """Set the `Turtlebot` to repeat its goals after `t` seconds. If `t <= 0`, the repeat timer is destroyed"""
        # checks if the repeat timer exists, if so, cancels it and destroys it
        if self.repeat_timer:
            self.repeat_timer.cancel()
            self.destroy_repeat_timer()
        if t > 0: # start the repeat timer if there is a meaningful repeat duration
            self.repeat_timer = self.create_timer(t, self.repeat_timer_callback, autostart=True)

    def home(self, x: float, y: float, theta: float) -> None:
        """Homes the `Turtlebot` to (`x`, `y`) with orientation `theta`"""
        self.bot.set_init_pose(x, y, theta)
        self.bot.initialize_pose()
        self.teleport(x, y, theta, verbose=False)

    def edit(self, data: List[float]) -> None:
        """
        Edit the goals of the `Turtlebot` with the given `data`, 
        a `list` of `float`s: `x1,y1,x2,y2,...`\n
        If the `loopback` parameter is `True`, the `Turtlebot` will return to the
        initial position and orientation after reaching all goals.\n
        If the `loopback` parameter is `False`, the `Turtlebot` will stop with the
        current orientation at the last goal.\n
        Note that the `Turtlebot` must be homed after an `edit` call
        """
        if len(data) % 2 != 0: # odd number of goals
            data = data[1:] # ignore the first element
        self.goals = self.param_to_goals(data)
        if self.loopback:
            self.set_goals_loopback(self.goals)
        else:
            self.set_goals(self.goals)
        self.get_logger().info(f'Edited goals: {self.goals}' + ' with loopback' if self.loopback else '')

    def stop(self) -> None:
        """Stop the `Turtlebot`"""
        self.paused_state = 'idle'
        self.bot.set_state('stop')

    def pause(self) -> None:
        """Pause the `Turtlebot`"""
        self.paused_state = self.bot.current # store the current state
        self.bot.set_state('idle')

    def resume(self) -> None:
        """Resume the `Turtlebot` from a paused state"""
        self.bot.set_state(self.paused_state)
        self.paused_state = 'idle' # reset the paused state

##################### Class Ended #####################

def main(args=None):
    """Main function to run the `Turtlebot` node"""
    rclpy.init(args=args) # initialize the ROS client library
    try:
        bot = Turtlebot()
        rclpy.spin(bot) # spin the Turtlebot
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