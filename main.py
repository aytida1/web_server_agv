from fastapi import FastAPI
from fastapi.responses import FileResponse  # for sending image file to client
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import DockRobot, UndockRobot
from typing import Any
import time

from map import HandleMap


from pathlib import Path   # object oriented way of hadling file paths

# global params
run_dock_loop = True
docking_request_future = None

class DockingAction(Node):
    def __init__(self):
        super().__init__('docking_action')
        self._action_client = ActionClient(
            self,
            DockRobot,
            '/agv1/dock_robot'
        )
    
    def send_docking_goal(self, dockId:str):
        dock_msg = DockRobot.Goal()
        dock_msg.use_dock_id = True
        dock_msg.dock_id = dockId

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return None
        
        dock_request_future = self._action_client.send_goal_async(dock_msg)
        return dock_request_future
    
    # this will return dock goal result STATUS
    def get_docking_result(self, future):
        #  check if goal accepted or not
        rclpy.spin_until_future_complete(self, future)
        dock_goal_handle = future.result()   # Now this is instance of ClientGoalHandle object class
        if not dock_goal_handle.accepted:
            self.get_logger().error("Goal rejected!!!!")
            return 3 # dock goal aborted
        
        self.get_logger().info("Goal accepted, waiting for result.......")

        dock_goal_result_future = dock_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, dock_goal_result_future)

        dock_result = dock_goal_result_future.result()   # this will return {status} property important for us

        return dock_result.status    # 2: CANCELLED, 3: ABORTED, 4: SUCCESS
    
    def cancel_docking_goal(self):
        cancel_dock_future = self.dock_goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_dock_future)
        self.get_logger().info("Dock CANCELLED !!!")


app = FastAPI()

rclpy.init()

# Create a single global instance to avoid multiple node registration
map_handler = HandleMap()
docking_client = DockingAction()


@app.get('/dock_loop_test')
def dock_loop_test():
    dockA = "test_dock1"
    dockB = "test_dock2"
    target_dock = dockA
    change_dock = 0   # if its even then choose dockA and if odd then choose dockB
    global run_dock_loop
    global docking_request_future
    while(run_dock_loop):
        if change_dock%2 == 0:
            target_dock = dockA
        else:
            target_dock = dockB

        docking_request_future = docking_client.send_docking_goal(target_dock)

        result = docking_client.get_docking_result(docking_request_future)
        if result == 4:
            time.sleep(8)
            change_dock += 1
            docking_client.get_logger().info("Docking SUCCESSFUL  :)")
        elif result == 3:
            docking_client.get_logger().error("Docking ABORTED  :(")
        elif result == 2:
            docking_client.get_logger().info("Goal CANCELLED  :(")
            run_dock_loop = False
            return {"data": "Docking cancelled!!"}


@app.get('/cancel_dock_goal')
def cancel_dock_goal():
    global docking_request_future
    goal_handle = docking_request_future.result()
    cancel_dock_future = goal_handle.cancel_docking_goal()
    rclpy.spin_until_future_complete(docking_client, cancel_dock_future)
    return {"data": "Dock cancelled !!!"}
        

@app.get('/test_dock1')
def docking():
    future = docking_client.send_docking_goal("test_dock2")
    rclpy.spin_until_future_complete(docking_client, future)
    return {"data": "Dock goal sent!!"}

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