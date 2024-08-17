import websockets
import asyncio
from rclpy.node import Node

class WebsocketNode(Node):
    def __init__(self, name: str = 'ws_node', 
                 address: str = 'localhost', port: int = 8765):
        super().__init__(name)
        self.name = name
        self.address = address
        self.port = port

    def msg_callback(self, msg) -> str:
        return 'Received: ' + msg

    # after receiving data from websocket, use msg_callback to handle the data
    async def websocket_handler(self, websocket, path):
        async for message in websocket:
            res = self.msg_callback(message)
            await websocket.send(res)

    def run(self):
        start_server = websockets.serve(self.websocket_handler, self.address, self.port)
        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()