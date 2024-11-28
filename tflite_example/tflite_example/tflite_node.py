import argparse
import time
import numpy as np
from PIL import Image
import tensorflow as tf
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
from ament_index_python.packages import get_package_share_directory


class TFLiteImageClassifier(Node):

    def __init__(self):
        super().__init__('tflite_image_classifier')

        image_path = '/tmp/grace_hopper.bmp'
        model_file_path = '/tmp/mobilenet_v1_1.0_224.tflite'
        label_file_path = '/tmp/labels.txt'

        self.declare_parameter('image', image_path)
        self.declare_parameter('model_file', model_file_path)
        self.declare_parameter('label_file', label_file_path)
        self.declare_parameter('input_mean', 127.5)
        self.declare_parameter('input_std', 127.5)
        self.declare_parameter('num_threads', None)
        self.declare_parameter('ext_delegate', None)
        self.declare_parameter('ext_delegate_options', '')

        self.publisher_ = self.create_publisher(String, 'classification_result', 10)

    def load_labels(self, filename):
        with open(filename, 'r') as f:
            return [line.strip() for line in f.readlines()]

    def classify_image(self):
        image_path = self.get_parameter('image').get_parameter_value().string_value
        model_file = self.get_parameter('model_file').get_parameter_value().string_value
        label_file = self.get_parameter('label_file').get_parameter_value().string_value
        input_mean = self.get_parameter('input_mean').get_parameter_value().double_value
        input_std = self.get_parameter('input_std').get_parameter_value().double_value

        # Load image
        img = Image.open(image_path)
        img = img.resize((224, 224))
        img_array = np.array(img) / 255.0
        img_array = (img_array - input_mean) / input_std
        img_array = np.expand_dims(img_array, axis=0).astype(np.float32)

        # Load labels
        labels = self.load_labels(label_file)

        # Load TFLite model
        interpreter = tf.lite.Interpreter(model_path=model_file)
        interpreter.allocate_tensors()

        # Get input/output tensors
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        # Set input tensor
        interpreter.set_tensor(input_details[0]['index'], img_array)

        # Run inference
        interpreter.invoke()

        # Get output tensor
        output_data = interpreter.get_tensor(output_details[0]['index'])
        predicted_class = np.argmax(output_data)

        # Publish result
        result_msg = String()
        result_msg.data = labels[predicted_class]
        self.publisher_.publish(result_msg)

        self.get_logger().info(f'Predicted Class: {labels[predicted_class]}')


def main(args=None):
    rclpy.init(args=args)
    tflite_classifier = TFLiteImageClassifier()

    # Continuously classify image at a fixed rate
    while rclpy.ok():
        tflite_classifier.classify_image()
        time.sleep(1)

    tflite_classifier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
