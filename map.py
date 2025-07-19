import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap

import numpy as np
import cv2 


class HandleMap(Node):
    def __init__(self):
        super().__init__('handle_map')
        self.get_map_client = self.create_client(GetMap, '/map_server/map')
        while not self.get_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again ...')
        self.req = GetMap.Request()
        
    def get_gridmap(self):
        self.future = self.get_map_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=5.0)
        return self.future.result()
    
    def save_map(self):
        response = self.get_gridmap()
        occupancy_grid = response.map
        if occupancy_grid:
            resolution = occupancy_grid.info.resolution
            width = occupancy_grid.info.width
            height = occupancy_grid.info.height
            data = occupancy_grid.data
            matrix_100 = np.empty((height, width))
            #converting to 2d array
            k = 0
            for i in range(height):
                for j in range(width):
                    matrix_100[i, (width-j-1)] = abs(data[k]-100)
                    k += 1

            matrix_255 = matrix_100 * 2.55
            matrix_255_int = np.clip(matrix_255, 0, 255).astype(np.uint8)
            cv2.imshow('i', matrix_255_int)
            cv2.imwrite('my_map.png', matrix_255_int)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        