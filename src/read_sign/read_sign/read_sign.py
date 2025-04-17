import rclpy
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import numpy as np
from read_sign.learn_signs import preprocess
import joblib
import cv2
from std_msgs.msg import Int32

class ReadSign(Node):
    def __init__(self):
        super().__init__('read_sign')

        # Create a custom QoS profile
        custom_qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_LAST,durability=QoSDurabilityPolicy.VOLATILE,depth=1)

        # Create the subscription
        self.subscription = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', # '/simulated_camera/image_raw/compressed' for sim
            self._image_callback, 
            custom_qos_profile
        )

        self.publisher_ = self.create_publisher(Int32, '/detected_sign', 10)

        self.model = joblib.load('./src/read_sign/saved_model_best.pkl')

        self.video_writer = None


    def _image_callback(self, img):

        # Convert compressed image to numpy array
        img = np.frombuffer(img.data, np.uint8)
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)

        # Show the image
        # if self.video_writer is None:
        #     height, width, _ = img.shape
        #     fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # use mp4v for mp4
        #     self.video_writer = cv2.VideoWriter('output.mp4', fourcc, 30.0, (width, height))
        #     if not self.video_writer.isOpened():
        #         print("Failed to open video writer!")
        #         return
            
        cv2.imshow('Original',img)
        key = cv2.waitKey(1) & 0xFF

        # Run img through model
        img,_ = preprocess(img)
        img = np.array(img).reshape(1,-1)
        prediction = int(self.model.predict(img))

        # Setup message to send
        msg = Int32()
        msg.data = prediction

        # Publish classification
        print(f'publishing prediction {msg.data}')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    read_sign = ReadSign()

    rclpy.spin(read_sign)

    read_sign.destroy_node()

    # read_sign.video_writer.release()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

