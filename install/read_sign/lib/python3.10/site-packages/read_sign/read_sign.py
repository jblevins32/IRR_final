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
            '/simulated_camera/image_raw/compressed', 
            self._image_callback, 
            custom_qos_profile
        )

        self.publisher_ = self.create_publisher(Int32, '/detected_sign', 10)

        self.model = joblib.load('./src/read_sign/saved_model_best.pkl')

    def _image_callback(self, img):

        # Convert compressed image to numpy array
        img = np.frombuffer(img.data, np.uint8)
        # frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Run img through model
        img,_ = preprocess(img)
        img = np.array(img).reshape(1,-1)
        prediction = int(self.model.predict(img))

        # Setup message to send
        msg = Int32()
        msg.data = prediction

        # Publish classification
        print(f'publishing prediction')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    read_sign = ReadSign()

    rclpy.spin(read_sign)

    read_sign.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

