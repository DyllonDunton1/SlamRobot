from nav_msgs.msg import OccupancyGrid  

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
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)  

        self.bridge = CvBridge()

        self.scan_data = []
        self.frame = np.zeros((240, 320, 3), dtype=np.uint8)

        self.map_surface = None  

        pygame.init()
        self.display = pygame.display.set_mode((1280, 480))  # 🔥 WIDER for 3 panels
        pygame.display.set_caption("Lidar + Camera + Map Display")
        self.clock = pygame.time.Clock()

    def scan_callback(self, msg):
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

    def map_callback(self, msg):  
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))

        # Convert occupancy grid to grayscale image
        image = np.zeros((height, width), dtype=np.uint8)
        image[data == 0] = 255       # Free = white
        image[data == 100] = 0       # Occupied = black
        image[data == -1] = 127      # Unknown = gray

        # Resize to fit panel (320x480), flip vertically so map shows upright
        image = cv2.flip(image, 0)
        resized = cv2.resize(image, (320, 480), interpolation=cv2.INTER_NEAREST)
        surface = pygame.surfarray.make_surface(np.stack([resized]*3, axis=-1).swapaxes(0, 1))
        self.map_surface = surface

    def draw(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

        self.display.fill((0, 0, 0))

        # Draw camera image (left)
        cam_surface = pygame.surfarray.make_surface(cv2.transpose(cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)))
        self.display.blit(cam_surface, (0, 0))

        # Draw LiDAR points (middle)
        lidar_surface = pygame.Surface((320, 480))
        lidar_surface.fill((0, 0, 0))
        for x, y in self.scan_data:
            sx = int(160 + x * 100)
            sy = int(240 - y * 100)
            pygame.draw.circle(lidar_surface, (0, 255, 0), (sx, sy), 2)
        self.display.blit(lidar_surface, (600, 0))

        # Draw map (right)
        if self.map_surface:
            self.display.blit(self.map_surface, (960, 0))

        pygame.display.flip()
        self.clock.tick(30)
        return True
