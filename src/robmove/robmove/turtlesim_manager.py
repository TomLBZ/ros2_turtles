import subprocess
from turtlesim.srv import Kill
from std_srvs.srv import Empty
from std_msgs.msg import String
from robmove.ws_node import WebsocketNode

class TurtlesimManager(WebsocketNode):
    def __init__(self, name: str = 'turtlebots_manager', 
                 address: str = 'localhost', port: int = 8765):
        super().__init__(name = name, address = address, port = port)
        nodelist = subprocess.check_output(['ros2', 'node', 'list'])
        strlist = nodelist.decode('utf-8').split('\n')
        names = list(set([name for s in strlist for name in s.split('/')]))
        if 'turtlesim' in names:
            self.get_logger().info('Turtlesim is detected')
        else:
            self.get_logger().info('Turtlesim is not detected, exiting...')
            exit(0)
        self.killifexists('turtle1')
        self.bots = {}
        self.publisher_cmds = {}
        self.n_sec = 0
        self.simultaneous = False
        self.auto_repeat = False
        self.countdown_timer = None
        self.counting = False
        self.repeat_timer = self.create_timer(120, self.repeat_timer_callback, autostart=False)
        
    def reset(self):
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

    def add_bot(self, bot):
        self.bots[bot.name] = bot
        self.publisher_cmds[bot.name] = self.create_publisher(String, bot.name + '/cmd', 10)

    def remove_bot(self, bot):
        del self.bots[bot.name]
        del self.publisher_cmds[bot.name]

    def repeat_timer_callback(self):
        self.start_task()
        if not self.auto_repeat:
            self.repeat_timer.cancel()

    def is_startable(self):
        ready = True
        for bot in self.bots.values():
            if bot.get_state() != 'idle':
                ready = False
                break
        return ready

    def countdown_timer_callback(self):
        self.n_sec -= 1
        if self.n_sec == 0:
            self.countdown_timer.cancel()
            self.countdown_timer.destroy()
            self.countdown_timer = None
            self.counting = False
            self.start_task()

    def publish_to_bot(self, botname, cmd):
        self.publisher_cmds[botname].publish(cmd)

    def start_task(self):
        if self.auto_repeat:
            self.repeat_timer.reset()
        if not self.is_startable():
            return
        if self.n_sec > 0:
            self.countdown_timer = self.create_timer(1, self.countdown_timer_callback, autostart=True)
            self.counting = True
            return
        for botname, bot in self.bots.items():
            self.publish_to_bot(botname, String(data='stop')) # stop the bot
            while bot.get_state() != 'idle':
                pass
            self.get_logger().info('Bot %s is idle and ready' % botname)
            self.publish_to_bot(botname, String(data='start')) # start the bot
            if not self.simultaneous:
                while bot.get_state() != 'idle':
                    pass # wait for the bot to finish and be ready

    def msg_callback(self, msg) -> str:
        # msg is a 1-1 mapping to the UI state
        # UI contains:
        # textbox: numeric, for countdown seconds
        # checkbox: boolean, 1 for simultaneous, 0 for sequential
        # checkbox: boolean, 1 for auto-repeat on, 0 for off
        # msg example: '60,1,0'
        # exit command
        if msg == 'exit':
            exit(0)
        msgs = msg.split(',')
        self.n_sec = int(msgs[0])
        self.simultaneous = bool(int(msgs[1]))
        self.auto_repeat = bool(int(msgs[2]))
        self.start_task()
        return msg