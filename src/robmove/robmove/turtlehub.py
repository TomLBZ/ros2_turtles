"""This file is designed to be run as a ROS2 node using the `ros2 run` command.
It is the main node that controls the whole system.\n
Note that all nodes are managed by this node, so DO NOT click on the [X]
or close their terminals / windows directly, as it may cause unexpected
behavior. Instead, press Ctrl+C in this terminal (where `TurtleHub` is run) 
to stop the subprocesses, and then (after a few seconds) press Ctrl+C again 
to stop this node gracefully.
"""

import subprocess
import signal
import rclpy
import asyncio
from asyncio import AbstractEventLoop, Task
from rclpy.node import Node, Subscription, Publisher, Timer
from turtlesim.srv import Kill
from std_srvs.srv import Empty
from std_msgs.msg import String, Bool
from robmove.turtlebot import Turtlebot
from typing import List, Dict, Self, Tuple, Set, Any

class TurtleHub(Node):
    """
    `TurtleHub` is the main node that controls the whole system, including:
    - a `turtlesim` node that provides a 2D simulation environment
    - a `ros2socket` node that provides a websocket server for communication
    - multiple `turtlebot` nodes that represent the robots in the simulation\n
    The `TurtleHub` node is responsible for:
    - starting the `turtlesim` and `ros2socket` nodes if they are not detected
    - resetting the `turtlesim` node if it is detected
    - managing the `turtlebot` nodes, dynamically adding or removing them
    - receiving commands from the `ros2socket` node and controlling the `turtlebot` nodes\n
    The `TurtleHub` node also publishes the state of the websocket connection
    and the state of the `turtlebot` nodes to the corresponding topics.
    """
    def __init__(self, sleep_ms: float = 0.0, loop: AbstractEventLoop = None) -> Self:
        """
        Instantiates a `TurtleHub` object with the given parameters:
        - `sleep_ms`: the sleep time in milliseconds for the main loop
        - `loop`: the asyncio event loop to run the main loop
        """
        self.name: str = 'turtlehub'
        super().__init__(self.name)
        # declares the parameters
        self.declare_parameters(namespace='', parameters=[
            ('output_topic', '/ros2socket/tosocket'),
            ('input_topic', '/ros2socket/fromsocket'),
            ('socket_state_topic', '/ros2socket/state'),
            ('state_topic', '/turtlehub/state'),
            ('state_publish_rate_hz', 10),
            ('ros2_bash', '/opt/ros/jazzy/setup.bash')
        ])
        # gets the parameters and assigns them to the corresponding variables
        self.output_topic: str = self.get_parameter('output_topic').value
        self.input_topic: str = self.get_parameter('input_topic').value
        self.state_topic: str = self.get_parameter('state_topic').value
        self.socket_state_topic: str = self.get_parameter('socket_state_topic').value
        self.state_publish_rate_hz: float = self.get_parameter('state_publish_rate_hz').value
        self.ros2_bash: str = self.get_parameter('ros2_bash').value
        # creates the publishers and the subscriber
        self.pub: Publisher = self.create_publisher(String, self.output_topic, 10)
        self.state_pub: Publisher = self.create_publisher(String, self.state_topic, 10)
        self.sub: Subscription = self.create_subscription(String, self.input_topic, self.msg_callback, 10)
        self.ros2socket_ready: bool = False
        self.sock_sub: Subscription = self.create_subscription(Bool, self.socket_state_topic, self.sock_callback, 10)
        # sets the period for the state publisher
        self.period: float = 1.0 / self.state_publish_rate_hz
        self.state_timer: Timer = self.create_timer(self.period, self.state_timer_callback)
        # prepare turtlesim
        self.turtlesim_ready: bool = False
        self.turtlesim_proc: subprocess.Popen = None
        if not self.node_exists('turtlesim'):
            self.get_logger().info('Turtlesim is not detected, starting turtlesim...')
            self.run_turtlesim()
        else:
            self.get_logger().info('Turtlesim is detected, resetting...')
            self.reset_turtlesim()
        self.get_logger().info('Turtlesim ready')
        self.turtlesim_ready = True
        # prepare ros2socket
        self.ros2socket_proc: subprocess.Popen = None
        if not self.node_exists('ros2socket'):
            self.get_logger().info('Ros2socket is not detected, starting ros2socket...')
            self.run_ros2socket()
        self.get_logger().info('Ros2socket ready')
        # waited appropriate time for turtle1 to be spawned, so kill it
        self.kill_if_exists('turtle1')
        # variables for managing the bots
        self.eventloop = loop if loop is not None else asyncio.get_event_loop()
        self.sleep_ms = sleep_ms
        self.botnames: List[str] = []
        self.bots: Dict[str, Turtlebot] = {}
        self.bot_states: Dict[str, str] = {}
        self.bot_subscribers: Dict[str, Subscription] = {}
        self.bots_asynctasks: Dict[str, Task] = {}
        self.task_running: bool = False
    
    ####### Subprocess Management #######

    def run_turtlesim(self) -> None:
        """Starts the `turtlesim` node as a subprocess"""
        # the command is 'ros2 run turtlesim turtlesim_node'
        self.turtlesim_proc: subprocess.Popen = subprocess.Popen(['ros2', 'run', 'turtlesim', 'turtlesim_node'])
        while not self.node_exists('turtlesim'):
            pass

    def run_ros2socket(self) -> None:
        """Starts the `ros2socket` node as a subprocess"""
        # the command is: ros2 run robmove ros2socket --ros-args --params-file config/ros2socket.yaml
        self.ros2socket_proc: subprocess.Popen = subprocess.Popen(['ros2', 'run', 'robmove', 'ros2socket', '--ros-args', '--params-file', 'config/ros2socket.yaml'])
        while not self.node_exists('ros2socket'):
            pass

    def terminate_subprocesses(self) -> None:
        """Terminates the `turtlesim` and `ros2socket` subprocesses by sending SIGINT signals"""
        if self.turtlesim_proc is not None:
            self.turtlesim_proc.send_signal(signal.SIGINT)
        if self.ros2socket_proc is not None:
            self.ros2socket_proc.send_signal(signal.SIGINT)

    ####### Bot Management #######
    
    def add_bot(self, bot: Turtlebot) -> None:
        """Adds a `Turtlebot` object to the list of bots and starts the bot asynchronously"""
        self.botnames.append(bot.name)
        self.bots[bot.name] = bot
        self.bot_states[bot.name] = "" # initialize to an unknown state
        # make the bot update its state when the state topic is updated
        def bot_state_callback(msg: String):
            self.bot_states[bot.name] = msg.data
        self.bot_subscribers[bot.name] = self.create_subscription(String, '/%s/state' % bot.name, bot_state_callback, 10)
        # start the bot in an asynchronous task
        self.bots_asynctasks[bot.name] = self.eventloop.create_task(self.run_bot(bot))

    def remove_bot(self, bot: Turtlebot) -> None:
        """Removes the given `bot` from the list of bots and stops the bot asynchronously"""
        name: str = bot.name
        if name not in self.botnames:
            self.get_logger().info('Bot %s not found' % name)
            return
        self.botnames.remove(name)
        self.bots.pop(name)
        self.bot_states.pop(name)
        self.destroy_subscription(self.bot_subscribers.pop(name))
        bot.kill()
        # cancel the asynchronous task and wait for it to finish
        self.bots_asynctasks[bot.name].cancel()
        self.eventloop.run_until_complete(self.bots_asynctasks[bot.name])
        self.bots_asynctasks.pop(bot.name)
        bot.destroy_node()

    async def run_bot(self, bot: Turtlebot) -> None:
        """Runs the given `bot` asynchronously"""
        try:
            while rclpy.ok():
                # run the bot without blocking the event loop
                rclpy.spin_once(bot)
                if self.sleep_ms >= 0:
                    await asyncio.sleep(self.sleep_ms)
        except KeyboardInterrupt:
            self.remove_bot(bot)
            # bot.kill()
            # bot.destroy_node()
        except Exception as e:
            readable_exception = e.__class__.__name__ + ': ' + str(e)
            self.get_logger().error('[run_bot] ' + readable_exception)

    ####### Helper Methods #######

    def is_all_states_idle(self) -> bool:
        """Checks if all the `Turtlebot` objects are in the idle state"""
        return all([bot.get_state() == 'idle' for bot in self.bots.values()])

    def gather_info(self) -> str:
        """Gathers the states of `TurtleHub`, `ros2socket`, `turtlesim` and `Turtlebot` objects as a string"""
        states: List[str] = [f'{name}:{state}' for name, state in self.bot_states.items()]
        states_str: str = '|'.join(states) # e.g. 'bot1:idle,...|bot2:turn0,...'
        socket_state: str = 'T' if self.ros2socket_ready else 'F'
        sim_state: str = 'T' if self.turtlesim_ready else 'F'
        self.task_running: bool = not self.is_all_states_idle()
        task_state: str = 'T' if self.task_running else 'F'
        return f'soc:{socket_state}|sim:{sim_state}|tsk:{task_state}|{states_str}'

    def is_valid_msg(self, msg: String) -> bool:
        """Checks if the given `msg` is in the valid format 'cmd:data'"""
        # split the message into lines
        lines: List[str] = msg.data.split('\n')
        # check if the message has only one line
        if len(lines) != 1:
            return False
        # make sure that only one colon is present in the message
        if msg.data.count(':') != 1:
            return False
        # make sure that the command and data are not empty
        cmd, data = msg.data.split(':')
        if cmd == '' or data == '':
            return False
        return True

    def node_exists(self, name: str) -> bool:
        """Checks if a node with the given `name` exists"""
        nodelist: bytes = subprocess.check_output(['ros2', 'node', 'list'])
        strlist: List[str] = nodelist.decode('utf-8').split('\n')
        names: List[str] = list(set([name for s in strlist for name in s.split('/')]))
        return name in names

    def reset_turtlesim(self) -> None:
        """Resets the `turtlesim` node by calling the `reset` service"""
        self.create_client(Empty, 'reset').call_async(Empty.Request())
        self.get_logger().info('Reset turtlesim')

    def kill_if_exists(self, name: str) -> None:
        """Kills the turtle with the given `name` if it exists"""
        topics: List[Tuple[str, List[str]]] = self.get_topic_names_and_types()
        topics: List[str] = [t for t, _ in topics]
        names: List[str] = list(set([name for topic in topics for name in topic.split('/')]))
        if name in names:
            killreq: Kill.Request = Kill.Request()
            killreq.name = name
            self.create_client(Kill, 'kill').call_async(killreq)
            self.get_logger().info('Killed %s' % name)
        else:
            self.get_logger().info('%s not found, skipped killing' % name)

    def str2bool(self, s: str) -> bool:
        """Converts the string `s` to a boolean value where only 'T' is converted to True"""
        return s == 'T'

    ####### Callbacks #######

    def sock_callback(self, msg: Bool) -> None:
        """Updates the state of the websocket connection using Bool `msg` on callback"""
        self.ros2socket_ready = msg.data

    def state_timer_callback(self) -> None:
        """Periodically publishes the state information gathered by `gather_info` on timer callback"""
        msg: String = String()
        msg.data = self.gather_info()
        self.state_pub.publish(msg)
        self.pub.publish(msg)

    def msg_callback(self, msg: String) -> None:
        """Handles the incoming String `msg` from the `ros2socket` node on callback"""
        try:
            cmd: str = msg.data
            if not self.is_valid_msg(msg):
                self.get_logger().info('Invalid message format: %s' % msg.data)
                return
            cmd, data = cmd.split(':')
            if cmd == 'spwn':
                self.on_spawn(data.split(','))
            elif cmd == 'kill':
                self.on_kill(data)
            elif cmd == 'strt':
                self.on_start(data.split(','))
            else:
                self.get_logger().info('Invalid command: %s' % cmd)
        except Exception as e:
            readable_exception = e.__class__.__name__ + ': ' + str(e)
            self.get_logger().error('[msg_callback] ' + readable_exception)

    ####### Command Handlers #######

    def on_spawn(self, data: List[str]) -> None:
        """
        Handles the 'spawn' command with the given `data` in the format:
        `"name,x,y,theta,loopback,p1x,p1y,...,pNx,pNy"` where:
        - `name` is the name of the bot,
        - `x,y` are the initial position of the bot,
        - `theta` is the initial orientation of the bot,
        - `loopback` is a boolean indicating whether the bot should 
            loop back to the starting point after reaching all the goal points,
        - `p1x,p1y,...,pNx,pNy` are the goal points
        """
        name, x, y, theta, loopback, pts = data[0], float(data[1]), float(data[2]), float(data[3]), self.str2bool(data[4]), [float(p) for p in data[5:]]
        if name in self.botnames: # bot already exists
            bot: Turtlebot = self.bots[name]
            bot.bot.drive(0.0, 0.0) # stops the bot
            bot.bot.set_state('idle') # set the state to idle
            bot.home(x, y, theta) # set the home position
            bot.loopback = loopback # set the loopback flag
            bot.edit(pts) # set the goal points
            self.bots[name] = bot # update the bot
        else:
            bot: Turtlebot = Turtlebot(name, x, y, theta, dependent=True)
            bot.loopback = loopback # set the loopback flag
            bot.edit(pts) # set the goal points
            self.add_bot(bot) # add the bot to the list

    def on_start(self, data: List[str]) -> None:
        """
        Handles the 'start' command with the given `data` in the format:
        `"t_after,t_repeat,is_repeat,is_simultaneous"` where:
        - `t_after` is the time after which the task starts,
        - `t_repeat` is the time after which the task repeats,
        - `is_repeat` is a boolean indicating whether the task should repeat,
        - `is_simultaneous` is a boolean indicating whether the task should 
            start all bots simultaneously\n
        Note that if any of the bot is not in idle state, the command will be ignored.
        """
        if not self.is_all_states_idle():
            self.get_logger().info('Task is already running, ignored command updates')
            return
        self.task_running: bool = True
        t_aft, t_rep, is_rep, is_simul = float(data[0]), float(data[1]), self.str2bool(data[2]), self.str2bool(data[3])
        t_rep = t_rep if is_rep else -1 # set the repeat time to -1 if not repeating
        # updates the bots
        prevbot = None
        for name in self.botnames:
            # clears the callback chain by setting the next callback to an empty lambda
            self.bots[name].bot.stop_callbacks['next'] = lambda: None
            # update the repeat time of the bot
            self.bots[name].repeat(t_rep)
            if is_simul: # start bots simultaneously
                self.bots[name].startin(t=t_aft)
            else: # start bots one after another
                if prevbot is not None: # previous bot has been set, second bot onwards
                    prevbot.bot.stop_callbacks['next'] = self.bots[name].start
                else: # the first bot
                    prevbot = self.bots[name]
                    prevbot.startin(t=t_aft)

    def on_kill(self, data: List[str]) -> None:
        """Handles the 'kill' command with the given `data` in the format: `"name"`"""
        name: str = data[0]
        if name in self.botnames:
            self.remove_bot(name)
        else:
            self.get_logger().info('Bot %s not found' % name)

#################### Class Ended ####################

# variable used to raise an exception when the loop is still running
loop_running_exception: BaseException = Exception('[stop_all_tasks: inner] Loop is still running')

def stop_all_tasks(loop: AbstractEventLoop) -> None:
    """Stops all the tasks in the given `loop`"""
    tasks: Set[Task[Any]] = asyncio.all_tasks(loop)
    for task in tasks:
        if task.done():
            continue
        if task.cancelling():
            while (task.cancelling() > 1):
                task.uncancel()
            continue
        if task.cancelled():
            continue
        # if the task has not been done or cancelled, cancel it
        task.cancel()
    try:
        if loop.is_running():
            raise loop_running_exception
        else: # get the results of the tasks and print any exceptions
            res: List[Any | BaseException] = loop.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))
            for r in res:
                if isinstance(r, KeyboardInterrupt):
                    print('[stop_all_tasks: inner] Detected KeyboardInterrupt')
                elif isinstance(r, Exception):
                    readable_r = '[stop_all_tasks: inner] ' + r.__class__.__name__ + ': ' + str(r)
                    print(readable_r)
    except Exception as e:
        # check if the exception is the one raised above
        if e == loop_running_exception:
            print('[stop_all_tasks: outer] Detected loop-still-running, trying to stop the loop...')
            print('[stop_all_tasks: outer] If the loop is not stopped after a few seconds, please press Ctrl+C again')
            loop.call_soon(loop.stop)
        readable_exception: str = '[stop_all_tasks: outer] ' + e.__class__.__name__ + ': ' + str(e)
        print(readable_exception)

async def manual_spin(node: TurtleHub, sleep_ms: float = 0.0, loop: AbstractEventLoop = None) -> None:
    """Manually spins the given `node` with the given `sleep_ms` and `loop`"""
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=sleep_ms)
            if sleep_ms >= 0:
                await asyncio.sleep(sleep_ms)
    except KeyboardInterrupt:
        if node is not None:
            node.terminate_subprocesses()
        if loop is not None:
            stop_all_tasks(loop)
    except Exception as e:
        readable_exception: str = e.__class__.__name__ + ': ' + str(e)
        node.get_logger().error('[manual_spin] ' + readable_exception)

def main() -> None:
    """The main function that runs the `TurtleHub` node"""
    rclpy.init()
    loop: AbstractEventLoop = asyncio.get_event_loop()
    try:
        hub: TurtleHub = TurtleHub(loop=loop)
        loop.create_task(manual_spin(hub, loop=loop))
        loop.run_forever()
    except KeyboardInterrupt:
        if hub is not None:
            hub.terminate_subprocesses()
        if loop is not None:
            stop_all_tasks(loop)
        if rclpy.ok():
            rclpy.shutdown()
        loop.stop()
        print('turtlehub gracefully stopped')
    except Exception as e:
        readable_exception: str = e.__class__.__name__ + ': ' + str(e)
        hub.get_logger().error('[main] ' + readable_exception)
    finally:
        loop.close()

if __name__ == '__main__':
    main()