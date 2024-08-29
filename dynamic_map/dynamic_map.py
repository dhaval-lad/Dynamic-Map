import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from tf_transformations import euler_from_quaternion

class DynamicMapUpdater(Node):

    def __init__(self):
        super().__init__('dynamic_map_updater')
        self.resolution = 0.05  # Example resolution (meters per cell)
        self.map_height = 100  # Example map height (cells)
        self.map_width = 50  # Example map width (cells)
        self.map_width_offset = 20  # Example map width offset (cells)
        self.origin_x = 0.0 - 0.263  # Example map origin x (meters) # 0.263 is subtracted due to laser is mounted 0.263m in X with respect to base link. 
        self.origin_y = 0.0 - ((self.map_width_offset // 2) * self.resolution)# Example map origin y (meters)
        self.local_map = np.zeros((self.map_width, self.map_height), dtype=np.int8)
        self.update_rate = 10  # Update rate in Hz
        self.odom_data = None
        self.scan_data = None
        self.expansion_size = 7  # Example expansion size

        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odom_callback,1)
        self.scan_sub = self.create_subscription(LaserScan,'/scan',self.scan_callback,qos_profile=qos_profile_sensor_data)

        self.publisher = self.create_publisher(OccupancyGrid,'/local_map',1)
        self.costmap_pub = self.create_publisher(OccupancyGrid,'/costmap',1)

        self.timer = self.create_timer(1.0 / self.update_rate, self.update_map)

    def scan_callback(self, msg):
        angles = np.arange(msg.angle_min, msg.angle_max , msg.angle_increment)
        ranges = np.array(msg.ranges)
        x = np.round((ranges * np.cos(angles) - self.origin_x) / self.resolution).astype(int) 
        y = np.round((ranges * np.sin(angles) - self.origin_y) / self.resolution).astype(int)
        # print(f"Length of ranges: {len(ranges)}, Length of angles: {len(angles)}")
        # print(f"Min angle: {msg.angle_min}, Max angle: {msg.angle_max}, Angle increment: {msg.angle_increment}")

        # Clear the map before updating
        self.local_map[:] = 0

        for i in range(len(x)):
            print(f"i={i}, x={x[i]}, y={y[i]}")
            if 0 <= x[i] < self.map_height and 0 <= y[i] < self.map_width:
                self.local_map[y[i], x[i]] = 100  # Mark as occupied
            else:
                print(f"Index out of bounds: x={x[i]}, y={y[i]}")

    def odom_callback(self, msg):
        # Update local map based on odometry data (robot's position)
        if self.odom_data is not None:
            # Clear previous position in the map
            prev_x = int((self.odom_data.pose.pose.position.x - self.origin_x) / self.resolution) 
            prev_y = int((self.odom_data.pose.pose.position.y - self.origin_y) / self.resolution)
            if 0 <= prev_x < self.map_height and 0 <= prev_y < self.map_width:
                self.local_map[prev_y, prev_x] = 0  # Clear previous position

        # Update current position
        self.odom_data = msg
        current_x = int((msg.pose.pose.position.x - self.origin_x) / self.resolution) 
        current_y = int((msg.pose.pose.position.y - self.origin_y) / self.resolution)
        if 0 <= current_x < self.map_height and 0 <= current_y < self.map_width:
            self.local_map[current_y, current_x] = -1  # Mark current position

    def costmap(self, data, width, height, resolution):
        # Extract yaw (rotation around z-axis) from quaternion
        orientation_q = self.odom_data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        data = np.array(data).reshape(height, width)
        wall = np.where(data == 100)
        for i in range(-self.expansion_size, self.expansion_size + 1):
            for j in range(-self.expansion_size, self.expansion_size + 1):
                if i == 0 and j == 0:
                    continue
                x = wall[0] + i
                y = wall[1] + j
                x = np.clip(x, 0, height - 1)
                y = np.clip(y, 0, width - 1)
                data[x, y] = 100
        data = data * resolution

        # Publish the costmap
        costmap_msg = OccupancyGrid()
        costmap_msg.header.frame_id = 'map'
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.info.resolution = resolution
        costmap_msg.info.width = width
        costmap_msg.info.height = height
        costmap_msg.info.origin.position.x = self.odom_data.pose.pose.position.x + (((self.map_width_offset // 2) * self.resolution)*np.sin(yaw))
        costmap_msg.info.origin.position.y = self.odom_data.pose.pose.position.y - (((self.map_width_offset // 2) * self.resolution)*np.cos(yaw))
        costmap_msg.info.origin.position.z = self.odom_data.pose.pose.position.z
        costmap_msg.info.origin.orientation.x = self.odom_data.pose.pose.orientation.x
        costmap_msg.info.origin.orientation.y = self.odom_data.pose.pose.orientation.y
        costmap_msg.info.origin.orientation.z = self.odom_data.pose.pose.orientation.z
        costmap_msg.info.origin.orientation.w = self.odom_data.pose.pose.orientation.w
        costmap_msg.data = (data.flatten() / resolution).astype(np.int8).tolist()  # Flatten the data and convert to int8

        self.costmap_pub.publish(costmap_msg)

        return data

    def update_map(self):
        # Extract yaw (rotation around z-axis) from quaternion
        orientation_q = self.odom_data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Periodically publish updated local map
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.map_height
        msg.info.height = self.map_width
        msg.info.origin.position.x = self.odom_data.pose.pose.position.x + (((self.map_width_offset // 2) * self.resolution)*np.sin(yaw))
        msg.info.origin.position.y = self.odom_data.pose.pose.position.y - (((self.map_width_offset // 2) * self.resolution)*np.cos(yaw))
        msg.info.origin.position.z = self.odom_data.pose.pose.position.z
        msg.info.origin.orientation.x = self.odom_data.pose.pose.orientation.x
        msg.info.origin.orientation.y = self.odom_data.pose.pose.orientation.y
        msg.info.origin.orientation.z = self.odom_data.pose.pose.orientation.z
        msg.info.origin.orientation.w = self.odom_data.pose.pose.orientation.w
        msg.data = np.ravel(self.local_map).tolist()
        self.publisher.publish(msg)

        # Generate the costmap and publish it
        self.costmap(self.local_map, self.map_height, self.map_width, self.resolution)


def main(args=None):
    rclpy.init(args=args)
    dynamic_map_updater = DynamicMapUpdater()
    rclpy.spin(dynamic_map_updater)
    dynamic_map_updater.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
