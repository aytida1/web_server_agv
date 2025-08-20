from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, HTMLResponse  # for sending image file to client
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import DockRobot, UndockRobot, NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener

from typing import Any
import time
import math

from map import HandleMap

from rclpy.task import Future
import asyncio
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from pathlib import Path   # object oriented way of hadling file paths

# global params
run_dock_loop = True
docking_request_future = None

class goalAction(Node):
    def __init__(self):
        super().__init__('goal_action')
        self.goal_action_client = ActionClient(
            self,
            NavigateToPose,
            '/agv1/navigate_to_pose'
        )
    
    def send_goal(self):
        goal_msg = NavigateToPose.Goal()

        # change me!
        my_pose = PoseStamped()
        my_pose.header.stamp = self.get_clock().now().to_msg()
        my_pose.header.frame_id = "map"
        my_pose.pose.position.x = 1.038
        my_pose.pose.position.y = 0.04
        my_pose.pose.orientation.x = 0.0
        my_pose.pose.orientation.y = -1.0
        my_pose.pose.orientation.z = 0.0
        my_pose.pose.orientation.w = 0.0058
        
        goal_msg.pose = my_pose
        goal_msg.behavior_tree = ""

        # sending goal request
        goal_request_future = self.goal_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_request_future)
        return goal_request_future
    
    def get_goal_result(self, future):
        # get future result whether it got accepted or not
        client_goal_handle = future.result()   # Now it is instance of ClientGoalHandle

        if client_goal_handle.accepted:
            self.get_logger().error('Navigation goal accepted!!')

            result_future = client_goal_handle.get_result_async()

            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()

            return result.status
        else:
            return 2 # goal cancelled



class DockingAction(Node):
    def __init__(self):
        super().__init__('docking_action')
        self._action_client = ActionClient(
            self,
            DockRobot,
            '/agv1/dock_robot'
        )

        self._undock_action_client = ActionClient(
            self,
            UndockRobot,
            '/agv1/undock_robot'
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "agv1/cmd_vel", 
            10
        )

        self.camera_detection_subscriber = self.create_subscription(
            AprilTagDetectionArray,
            "/agv1/detections",
            self.detection_callback,
            10
        )

        self.dock_goal_handle = None

        self.heading_timer = None

        self.desired_centre_x = 320.0
        self.target_frame = "tag36h11:0"
        self.source_frame = "agv1/base_link"
        self.centre_x = 0.0

        # # for getting transform
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

    def detection_callback(self, det_msg):
        if det_msg.detections:
            self.centre_x = det_msg.detections[0].centre.x  # 640/2 = 320

    def undock_from_dock(self, dockType:str):
        undock_msg = UndockRobot.Goal()
        undock_msg.dock_type = dockType
        undock_msg.max_undocking_time = 40.0

        if not self._undock_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Undocking server not available.')
            return None
        
        undock_request_future = self._undock_action_client.send_goal_async(undock_msg)
        rclpy.spin_until_future_complete(self, undock_request_future)

        undock_goal_handle = undock_request_future.result()

        if not undock_goal_handle.accepted:
            self.get_logger().error('Undocking request NOT accepted.')

        self.get_logger().info('Undocking request accepted')

        undock_result_future = undock_goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, undock_result_future)

        undock_result = undock_result_future.result().result

        if undock_result.success == True:
            self.get_logger().info('Undocked successful!!')
            return True
        else:
            return False

    def send_docking_goal(self, dockId:str):
        dock_msg = DockRobot.Goal()
        dock_msg.use_dock_id = True
        dock_msg.dock_id = dockId
        dock_msg.max_staging_time = 1000.0
        dock_msg.navigate_to_staging_pose = False

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return None
        
        dock_request_future = self._action_client.send_goal_async(dock_msg)
        rclpy.spin_until_future_complete(self, dock_request_future)
        return dock_request_future
    
    # this will return dock goal result STATUS
    def get_docking_result(self, future):
        #  check if goal accepted or not
        
        self.dock_goal_handle = future.result()   # Now this is instance of ClientGoalHandle object class
        if not self.dock_goal_handle.accepted:
            self.get_logger().error("Goal rejected!!!!")
            return 3 # dock goal aborted
        
        self.get_logger().info("Goal accepted, waiting for result.......")

        dock_goal_result_future = self.dock_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, dock_goal_result_future)

        dock_result = dock_goal_result_future.result()   # this will return {status} property important for us

        return dock_result.status    # 2: CANCELLED, 3: ABORTED, 4: SUCCESS
    
    def resolve_heading_error(self):
        if self.heading_timer is None:
            self.get_logger().info("Yaw correction started!!")

            self._yaw_future = Future()  # this is member parameter

            self.heading_timer = self.create_timer(0.1, self.resolve_heading_error_step)

            return self._yaw_future

            

        # more straight forward approach using while loop 
        # while True:
        #     error = self.desired_centre_x - self.centre_x
        #     print(error)
        #     if abs(error) <= 5.0:
        #         self.get_logger().info("done docking !! Heading aligned!!")
        #         self.cmd_vel_pub.publish(Twist())  # stop the robot
        #         # stop the timer
        #         # self.heading_timer.cancel()
        #         # self.heading_timer = None
        #         break

        #     # angular vel commands
        #     cmd = Twist()
        #     cmd.angular.z = 0.03 if error > 5.0 else -0.03
        #     self.cmd_vel_pub.publish(cmd)

        #     time.sleep(0.1)
        
                
    def resolve_heading_error_step(self):
        # handle not getting transform pose
        # try:
        #     t = self.tf_buffer.lookup_transform(
        #         self.target_frame,
        #         self.source_frame,
        #         rclpy.time.Time()
        #     )
        # except TransformException as ex:
        #     self.get_logger().error(f"Could not transform {self.source_frame} and {self.target_frame}: {ex}")
        #     stop_cmd = Twist()
        #     stop_cmd.linear.x = 0.0
        #     stop_cmd.angular.z = 0.0
        #     # self.vel_pub.publish(stop_cmd)
        #     return
            
        # x_ = t.transform.translation.y

        # yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        # yaw = math.atan2(2 * ((q.y * q.z + q.x * q.w)), q.x**2 + q.y**2 - q.z**2 - q.w**2)
        # print(x_)

        # error = yaw - self.desired_yaw
        error = self.desired_centre_x - self.centre_x

        if abs(error) <= 10.0:
            self.get_logger().info("done docking !! Heading aligned !!")
            self.cmd_vel_pub.publish(Twist())  # stop the robot
            # stop the timer
            self.heading_timer.cancel()
            self.heading_timer = None

            if not self._yaw_future.done():
                self._yaw_future.set_result(True)
            return

        # angular vel commands
        cmd = Twist()
        cmd.angular.z = 0.03 if error > 10.0 else -0.03
        self.cmd_vel_pub.publish(cmd)

    
    def cancel_docking_goal(self):
        if self.dock_goal_handle == None:
            self.get_logger().error("No active docking action going on!!")
            return
        cancel_dock_future = self.dock_goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_dock_future)
        self.get_logger().info("Dock CANCELLED !!!")


app = FastAPI()

# mount static files
app.mount("/static", StaticFiles(directory="static"), name="static")

# Setup Jinja2 template engine
templates = Jinja2Templates(directory="templates")

rclpy.init()

# Create a single global instance to avoid multiple node registration
map_handler = HandleMap()
docking_client = DockingAction()
navigation_goal_client = goalAction()

# executor = MultiThreadedExecutor()
# executor.add_node(docking_client)

# def ros_spin():
#     executor.spin()

# Thread(target=ros_spin, daemon=True).start()

@app.get("/", response_class=HTMLResponse)
async def root(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})


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
            time.sleep(11)
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
    cancel_dock_future = goal_handle.cancel_goal_async()
    rclpy.spin_until_future_complete(docking_client, cancel_dock_future)
    return {"data": "Dock cancelled !!!"}
        

@app.get('/test_dock1')
async def docking():
    # first send staging pose goal
    goal_req_future = navigation_goal_client.send_goal()
    result_code = navigation_goal_client.get_goal_result(goal_req_future)

    if result_code == 4:
        navigation_goal_client.get_logger().error('Successfully reached staging pose.')
        # some delay
        await asyncio.sleep(2.0)

        # sending dock request
        future = docking_client.send_docking_goal("test_dock1")
        dock_result_code = docking_client.get_docking_result(future)

        # check for docking succeed
        if dock_result_code == 4:
            time.sleep(1.5)

            yaw_future = docking_client.resolve_heading_error()
            rclpy.spin_until_future_complete(docking_client, yaw_future)

            return {"data": "Docking done!!"}   
        
@app.get('/undock')
async def undock():
    done_undock = docking_client.undock_from_dock("office_dock1")
    if done_undock:
        return {"data": "Undocking done!!"}
    else:
        return {"data": "Undocking NOT done!!!!"}
        

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