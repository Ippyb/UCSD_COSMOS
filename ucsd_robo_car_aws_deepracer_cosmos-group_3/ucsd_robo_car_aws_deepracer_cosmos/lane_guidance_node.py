import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, Int32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

# Nodes in this program
NODE_NAME = 'lane_detection_node'

# Topics subcribed/published to in this program
CAMERA_TOPIC_NAME = '/camera_pkg/display_mjpeg'
CENTROID_TOPIC_NAME = '/centroid'


class LaneDetection(Node):
    def __init__(self):
        # Initializing LaneDetection class
        super().__init__(NODE_NAME)
        self.centroid_error_publisher = self.create_publisher(Float32, CENTROID_TOPIC_NAME, 10)
        self.centroid_error_publisher
        self.centroid_error = Float32()
        self.camera_subscriber = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.custom_controller, 10)
        self.camera_subscriber
        self.bridge = CvBridge()

        # Declaring parameters (can delete if needed)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('parameter_1', 1),
                ('parameter_2', 1),
                ('parameter_3', 1)
            ])
        self.parameter_1 = self.get_parameter('parameter_1').value
        self.parameter_2 = self.get_parameter('parameter_2').value
        self.parameter_3 = self.get_parameter('parameter_3').value

        # Printing parameters
        self.get_logger().info(
            f'\nparameter_1: {self.parameter_1}'
            f'\nparameter_2: {self.parameter_2}'
            f'\nparameter_3: {self.parameter_3}')

    def custom_controller(self, data):
        # Get image feed from camera
        frame = self.bridge.imgmsg_to_cv2(data)

        '''
        your custom controller goes here...

        '''

        # Publish an error value
        self.centroid_error.data = float(error_x)
        self.centroid_error_publisher.publish(self.centroid_error)

        # Show image
        cv2.imshow('frame', frame)


def main(args=None):
    rclpy.init(args=args)
    centroid_publisher = LaneDetection()
    try:
        rclpy.spin(centroid_publisher)
        centroid_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        centroid_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        cv2.destroyAllWindows()
        centroid_publisher.destroy_node()
        rclpy.shutdown()
        centroid_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
