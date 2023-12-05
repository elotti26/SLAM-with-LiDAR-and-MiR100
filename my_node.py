import pysicktim as lidar
import rclpy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import ColorRGBA
import time

def publish_lidar_data(timer, publisher):
    try:
        lidar.scan()
        d = lidar.scan.distances

        num_points = len(d)
        angle_increment = (270.0 * 3.14159265 / 180.0) / num_points  # Calculate angle increment
        
        # Duplica l'ultima lettura per ottenere 272 letture
        d.append(d[-1])

        # Create a LaserScan message
        scan_msg = LaserScan()
        scan_msg.header.frame_id = 'scan'  # Adjust the frame_id as needed
        scan_msg.header.stamp = rclpy.time.Time().to_msg()
        #scan_msg.angle_min = -142.0 * 3.14159265 / 180.0
        #scan_msg.angle_max = 123.0 * 3.14159265 / 180.0  # 270 degrees in radians
        scan_msg.angle_min = -130.0 * 3.14159265 / 180.0
        scan_msg.angle_max = 140.0 * 3.14159265 / 180.0  # 270 degrees in radians
        scan_msg.angle_increment = angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.range_min = 0.0  # Minimum detectable distance
        scan_msg.range_max = 20.0  # Maximum detectable distance
        scan_msg.ranges = d

        # Publish the LiDAR data
        publisher.publish(scan_msg)

        # Print the detected distances to the terminal
        print("Detected Distances (in meters):", d)
    except ValueError as e:
        print(f"ValueError occurred: {e}. Continuing scanning...")

def publish_footprint_marker(publisher):
    marker_msg = Marker()
    marker_msg.header.frame_id = 'base_footprint'  # Usa il frame appropriato
    marker_msg.header.stamp = rclpy.time.Time().to_msg()
    marker_msg.ns = 'robot_footprint'
    marker_msg.id = 0
    marker_msg.type = Marker.LINE_STRIP
    marker_msg.action = Marker.ADD
    marker_msg.pose.orientation.w = 1.0
    marker_msg.scale.x = 0.02  # Spessore del poligono nel display

    # Definisci i punti del poligono che rappresenta il footprint del robot
    # Considera le dimensioni e la forma del robot per calcolare i punti
    points = [
        Point(x=0.4375, y=0.28, z=0.0),  # Esempio: Adatta i punti in base alle dimensioni
        Point(x=-0.4375, y=0.28, z=0.0),
        Point(x=-0.4375, y=-0.28, z=0.0),
        Point(x=0.4375, y=-0.28, z=0.0),
        Point(x=0.4375, y=0.28, z=0.0),  # Ripeti il primo punto per chiudere il poligono
    ]
    marker_msg.points = points
    
    # Imposta il colore rosso
    red = ColorRGBA()
    red.r = 1.0
    red.g = 0.0
    red.b = 0.0
    red.a = 1.0

    marker_msg.colors = [red] * len(points)  # Imposta il colore rosso per ogni punto nel poligono

    publisher.publish(marker_msg)
def main():
    rclpy.init()
    node = rclpy.create_node('lidar_publisher_node')

    # Create a publisher for the LiDAR data
    publisher = node.create_publisher(LaserScan, 'scan', 10)

    # Set the publishing rate to 1 Hz (once per second)
    publish_rate = 1  # Hz
    timer_period = 1.0 / publish_rate
    timer = node.create_timer(timer_period, lambda: publish_lidar_data(timer, publisher))

    # Publisher for the footprint marker
    footprint_publisher = node.create_publisher(Marker, 'robot_footprint', 10)

    try:
        while rclpy.ok():
            publish_footprint_marker(footprint_publisher)
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    
