import subprocess
import signal
import rclpy
import asyncio
from rclpy.node import Node, Subscription
from turtlesim.srv import Kill
from std_srvs.srv import Empty
from std_msgs.msg import String, Bool
from robmove.turtlebot import Turtlebot
from typing import List, Dict

class TurtleHub(Node):
    def __init__(self, sleep_ms: float = 0.0, loop: asyncio.AbstractEventLoop = None):
        self.name = 'turtlehub'
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
        self.state_publish_rate_hz: int = self.get_parameter('state_publish_rate_hz').value
        self.ros2_bash: str = self.get_parameter('ros2_bash').value
        # creates the publishers and the subscriber
        self.pub = self.create_publisher(String, self.output_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.sub = self.create_subscription(String, self.input_topic, self.msg_callback, 10)
        self.ros2socket_ready: bool = False
        self.sock_sub = self.create_subscription(Bool, self.socket_state_topic, self.sock_callback, 10)
        # sets the period for the state publisher
        self.period: float = 1.0 / self.state_publish_rate_hz
        self.state_timer = self.create_timer(self.period, self.state_timer_callback)
        # prepare turtlesim
        self.turtlesim_ready: bool = False
        self.turtlesim_proc = None
        if not self.node_exists('turtlesim'):
            self.get_logger().info('Turtlesim is not detected, starting turtlesim...')
            self.run_turtlesim()
        else:
            self.get_logger().info('Turtlesim is detected, resetting...')
            self.reset_turtlesim()
        self.get_logger().info('Turtlesim ready')
        self.turtlesim_ready = True
        # prepare ros2socket
        self.ros2socket_proc = None
        if not self.node_exists('ros2socket'):
            self.get_logger().info('Ros2socket is not detected, starting ros2socket...')
            self.run_ros2socket()
        self.get_logger().info('Ros2socket ready')
        # wait appropriate time for turtle1 to be spawned, then kill it
        self.killifexists('turtle1')
        # variables for managing the bots
        self.eventloop = loop if loop is not None else asyncio.get_event_loop()
        self.sleep_ms = sleep_ms
        self.botnames: List[str] = []
        self.bots: Dict[str, Turtlebot] = {}
        self.bot_states: Dict[str, str] = {}
        self.bot_subscribers: Dict[str, Subscription] = {}
        self.bots_asynctasks: Dict[str, asyncio.Task] = {}
        self.task_running: bool = False
    
    # callback for the ros2socket state
    def sock_callback(self, msg: Bool):
        self.ros2socket_ready = msg.data

    # periodically publish the state of the websocket connection
    def state_timer_callback(self):
        msg = String()
        msg.data = self.gather_info()
        self.state_pub.publish(msg)
        self.pub.publish(msg)

    def gather_info(self) -> str:
        states: List[str] = [f'{name}:{state}' for name, state in self.bot_states.items()]
        states_str = '|'.join(states) # e.g. 'bot1:idle,...|bot2:turn0,...'
        socket_state = 'T' if self.ros2socket_ready else 'F'
        sim_state = 'T' if self.turtlesim_ready else 'F'
        is_all_states_idle = all([bot.get_state() == 'idle' for bot in self.bots.values()])
        self.task_running = not is_all_states_idle
        task_state = 'T' if self.task_running else 'F'
        return f'soc:{socket_state}|sim:{sim_state}|tsk:{task_state}|{states_str}'

    def is_valid_msg(self, msg: String) -> bool:
        # the format should be a single line 'cmd:data'
        # example: 'spwn:trigbot,6.0,3.0,0.0,T,9.0,7.0,7.0,6.0'
        # example: 'kill:rectbot'
        # split the message into lines
        lines = msg.data.split('\n')
        # check if the message has only one line
        if len(lines) != 1:
            return False
        # make sure that only one colon is present
        if msg.data.count(':') != 1:
            return False
        # make sure that the command and data are not empty
        cmd, data = msg.data.split(':')
        if cmd == '' or data == '':
            return False
        return True

    def msg_callback(self, msg: String) -> str:
        try:
            cmd = msg.data
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
        
    def on_spawn(self, data: List[str]):
        name, x, y, theta, loopback, pts = data[0], float(data[1]), float(data[2]), float(data[3]), bool(data[4]), [float(p) for p in data[5:]]
        if name in self.botnames:
            bot: Turtlebot = self.bots[name]
            bot.bot.drive(0.0, 0.0) # stops the bot
            bot.bot.set_state('idle') # set the state to idle
            bot.home(x, y, theta)
            bot.loopback = loopback
            bot.edit(pts)
            self.bots[name] = bot
        else:
            bot = Turtlebot(name, x, y, theta, dependent=True)
            bot.loopback = loopback
            bot.edit(pts)
            self.add_bot(bot)

    def on_kill(self, data: List[str]):
        name = data[0]
        if name in self.botnames:
            self.remove_bot(name)
        else:
            self.get_logger().info('Bot %s not found' % name)

    def on_start(self, data: List[str]):
        is_all_states_idle = all([bot.get_state() == 'idle' for bot in self.bots.values()])
        if not is_all_states_idle:
            self.get_logger().info('Task is already running, ignored command updates')
            return
        self.task_running = True
        def str2bool(s: str) -> bool:
            return s == 'T'
        t_after, t_repeat, is_repeat, is_simultaneous = float(data[0]), float(data[1]), str2bool(data[2]), str2bool(data[3])
        # updates the bots
        def empty_func():
            pass
        prevbot = None
        for name in self.botnames:
            self.bots[name].bot.stop_callbacks['next'] = empty_func # clear the callback chain
            if is_repeat: # set the repeat countdown
                self.bots[name].repeat(t_repeat)
            if is_simultaneous: # start bots simultaneously
                self.bots[name].startin(t=t_after)
            else: # start bots one after another
                if prevbot is not None: # previous bot has been set, second bot onwards
                    prevbot.bot.stop_callbacks['next'] = self.bots[name].start
                else: # the first bot
                    prevbot = self.bots[name]
                    prevbot.startin(t=t_after)

    def node_exists(self, name: str) -> bool:
        nodelist = subprocess.check_output(['ros2', 'node', 'list'])
        strlist = nodelist.decode('utf-8').split('\n')
        names = list(set([name for s in strlist for name in s.split('/')]))
        return name in names

    def run_turtlesim(self):
        # the command is 'ros2 run turtlesim turtlesim_node'
        self.turtlesim_proc = subprocess.Popen(['ros2', 'run', 'turtlesim', 'turtlesim_node'])
        while not self.node_exists('turtlesim'):
            pass

    def run_ros2socket(self):
        # the command is: ros2 run robmove ros2socket --ros-args --params-file config/ros2socket.yaml
        self.ros2socket_proc = subprocess.Popen(['ros2', 'run', 'robmove', 'ros2socket', '--ros-args', '--params-file', 'config/ros2socket.yaml'])
        while not self.node_exists('ros2socket'):
            pass

    def reset_turtlesim(self):
        self.create_client(Empty, 'reset').call_async(Empty.Request())
        self.get_logger().info('Reset turtlesim')

    def killifexists(self, name):
        topics = self.get_topic_names_and_types()
        topics = [t for t, _ in topics]
        names = list(set([name for topic in topics for name in topic.split('/')]))
        if name in names:
            killreq = Kill.Request()
            killreq.name = name
            self.create_client(Kill, 'kill').call_async(killreq)
            self.get_logger().info('Killed %s' % name)
        else:
            self.get_logger().info('%s not found, skipped killing' % name)

    async def run_bot(self, bot: Turtlebot):
        try:
            while rclpy.ok():
                rclpy.spin_once(bot)
                if self.sleep_ms >= 0:
                    await asyncio.sleep(self.sleep_ms)
        except KeyboardInterrupt:
            bot.kill()
            bot.destroy_node()
        except Exception as e:
            readable_exception = e.__class__.__name__ + ': ' + str(e)
            self.get_logger().error('[run_bot] ' + readable_exception)

    def add_bot(self, bot: Turtlebot):
        self.botnames.append(bot.name)
        self.bots[bot.name] = bot
        self.bot_states[bot.name] = ""
        def bot_state_callback(msg: String):
            self.bot_states[bot.name] = msg.data
        self.bot_subscribers[bot.name] = self.create_subscription(String, '/%s/state' % bot.name, bot_state_callback, 10)
        self.bots_asynctasks[bot.name] = self.eventloop.create_task(self.run_bot(bot))

    def remove_bot(self, bot: Turtlebot):
        name = bot.name
        if name not in self.botnames:
            self.get_logger().info('Bot %s not found' % name)
            return
        self.botnames.remove(name)
        self.bots.pop(name)
        self.bot_states.pop(name)
        self.destroy_subscription(self.bot_subscribers.pop(name))
        bot.kill()
        self.bots_asynctasks[bot.name].cancel()
        self.eventloop.run_until_complete(self.bots_asynctasks[bot.name])
        self.bots_asynctasks.pop(bot.name)
        bot.destroy_node()

    def terminate_subprocesses(self):
        if self.turtlesim_proc is not None:
            self.turtlesim_proc.send_signal(signal.SIGINT)
        if self.ros2socket_proc is not None:
            self.ros2socket_proc.send_signal(signal.SIGINT)

def stop_all_tasks(loop: asyncio.AbstractEventLoop):
    tasks = asyncio.all_tasks(loop)
    for task in tasks:
        task.cancel()
    try:
        if loop.is_running():
            pass
        else:
            res = loop.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))
            for r in res:
                if isinstance(r, Exception):
                    readable_r = r.__class__.__name__ + ': ' + str(r)
                    print(readable_r)
    except Exception as e:
        readable_exception = e.__class__.__name__ + ': ' + str(e)
        print(readable_exception)

async def manual_spin(node: TurtleHub, sleep_ms: float = 0.0, loop: asyncio.AbstractEventLoop = None):
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
        readable_exception = e.__class__.__name__ + ': ' + str(e)
        node.get_logger().error('[manual_spin] ' + readable_exception)

def main():
    rclpy.init()
    loop = asyncio.get_event_loop()
    hub = TurtleHub(loop=loop)
    try:
        loop.create_task(manual_spin(hub, loop=loop))
        loop.run_forever()
    except KeyboardInterrupt:
        hub.terminate_subprocesses()
        stop_all_tasks(loop)
        if rclpy.ok():
            rclpy.shutdown()
        loop.stop()
        print('turtlehub gracefully stopped')
    except Exception as e:
        readable_exception = e.__class__.__name__ + ': ' + str(e)
        hub.get_logger().error('[main] ' + readable_exception)
    finally:
        loop.close()

if __name__ == '__main__':
    main()