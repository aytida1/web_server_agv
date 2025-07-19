from fastapi import FastAPI
from fastapi.responses import FileResponse  # for sending image file to client
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import DockRobot, UndockRobot

from map import HandleMap


from pathlib import Path   # object oriented way of hadling file paths

class DockingAction(Node):
    def __init__(self):
        super().__init__('docking_action')
        self._action_client = ActionClient(
            self,
            DockRobot,
            '/agv1/dock_robot'
        )
    
    def send_docking_goal(self):
        dock_msg = DockRobot.Goal()
        dock_msg.use_dock_id = True
        dock_msg.dock_id = "test_dock"

        # self._action_client.wait_for_server(5)

        return self._action_client.send_goal_async(dock_msg)

app = FastAPI()

rclpy.init()

# Create a single global instance to avoid multiple node registration
map_handler = HandleMap()

@app.get('/test_dock1')
def docking():
    docking_client = DockingAction()
    future = docking_client.send_docking_goal()
    rclpy.spin_until_future_complete(docking_client, future)
    return {'data': "Dock goal sent!!"}

@app.get('/save_map')
async def save_map():
    map_handler.save_map()
    return {"data": "map saved."}

@app.get('/get_map')
async def send_map():
    image_path = Path("my_map.png")    # now my task is to give path to actual map png file from database or something
    if not image_path.is_file():
        return {"error": "Image not found on the server"}
    return FileResponse(image_path)