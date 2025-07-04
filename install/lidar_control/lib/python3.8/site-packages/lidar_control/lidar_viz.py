import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import pygame
import cv2

class LidarViz(Node):
    def __init__(self):
        super().__init__('lidar_viz')

        self.qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, self.qos)
        self.create_subscription(Image, '/image_raw', self.image_callback, self.qos)

        # CV bridge
        self.bridge = CvBridge()

        # LiDAR data
        self.scan_data = []

        # Image data
        self.frame = np.zeros((240, 320, 3), dtype=np.uint8)  # fallback blank image

        # Pygame setup
        pygame.init()
        self.display = pygame.display.set_mode((920, 480))
        pygame.display.set_caption("Lidar + Camera Display")
        self.clock = pygame.time.Clock()

        

    def scan_callback(self, msg):
        print(msg)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.clip(np.array(msg.ranges), 0.0, 3.5)
        self.scan_data = [
            (r * np.cos(a), r * np.sin(a)) for r, a in zip(ranges, angles)
        ]

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame = cv2.resize(cv_image, (600, 480))
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def draw(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

        self.display.fill((0, 0, 0))

        # Draw camera image
        cam_surface = pygame.surfarray.make_surface(cv2.transpose(cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)))
        self.display.blit(cam_surface, (0, 0))

        # Draw LiDAR points (centered on right half)
        lidar_surface = pygame.Surface((320, 480))
        lidar_surface.fill((0, 0, 0))
        for x, y in self.scan_data:
            sx = int(160 + x * 100)  # center in 320px area
            sy = int(240 - y * 100)
            pygame.draw.circle(lidar_surface, (0, 255, 0), (sx, sy), 2)
        self.display.blit(lidar_surface, (600, 0))

        pygame.display.flip()
        self.clock.tick(30)
        return True

def main(args=None):
    rclpy.init(args=args)
    node = LidarViz()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if not node.draw():
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
