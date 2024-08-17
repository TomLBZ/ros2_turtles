from typing import Tuple
from threading import Thread
import rclpy
from rclpy.executors import MultiThreadedExecutor
from robmove.turtlesim_manager import TurtlesimManager
from robmove.turtlebot import Turtlebot

def init() -> Tuple[TurtlesimManager, MultiThreadedExecutor]:
    rclpy.init()
    manager = TurtlesimManager()
    rectbot = Turtlebot('rectrob', 1.0, 4.0)
    rectbot.set_goals_loopback([(4.0, 4.0), (4.0, 6.0), (1.0, 6.0)])
    trigbot = Turtlebot('trirob', 6.0, 3.0)
    trigbot.set_goals_loopback([(9.0, 7.0), (7.0, 6.0)])
    manager.add_bot(rectbot)
    manager.add_bot(trigbot)
    executor = MultiThreadedExecutor()
    executor.add_node(manager)
    executor.add_node(manager.bots['rectrob'])
    executor.add_node(manager.bots['trirob'])
    return manager, executor

def main(args=None):
    manager, executor = init()
    # spin in a separate thread
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    manager.run()
    # # pools for keyboard input
    # while True:
    #     try:
    #         x = input('Enter a command: ')
    #         if x == 'exit':
    #             rectbot.pause()
    #             trigbot.pause()
    #             rectbot.stop()
    #             trigbot.stop()
    #             break
    #         elif x == 'rect':
    #             rectbot.start()
    #         elif x == 'tri':
    #             trigbot.start()
    #         elif x == 'both':
    #             rectbot.start()
    #             trigbot.start()
    #         elif x == 'pause':
    #             rectbot.pause()
    #             trigbot.pause()
    #         elif x == 'resume':
    #             rectbot.resume()
    #             trigbot.resume()
    #         elif x == 'stop':
    #             rectbot.pause()
    #             trigbot.pause()
    #             rectbot.stop()
    #             trigbot.stop()
    #         else:
    #             print('Invalid command')
    #     except EOFError:
    #         break
    executor.shutdown(timeout_sec=1.0)
    executor_thread.join(timeout=1.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()