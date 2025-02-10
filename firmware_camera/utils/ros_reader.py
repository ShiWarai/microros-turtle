import rclpy
from rclpy.node import Node
from sensor_micro_msgs.msg import CompressedImage
import cv2
import numpy as np

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.subscription
        self.get_logger().info('Image Viewer Node has started')

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image_np is not None:
            # Rotate the image 90 degrees clockwise
            image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
            
            cv2.imshow("ESP32-CAM Image", image_np)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    image_viewer = ImageViewer()

    try:
        rclpy.spin(image_viewer)
    except KeyboardInterrupt:
        print("ERROR")

    image_viewer.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
