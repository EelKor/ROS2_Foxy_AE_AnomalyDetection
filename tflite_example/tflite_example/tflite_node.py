import rclpy
from rclpy.node import Node
import numpy as np
import tflite_runtime.interpreter as tflite

class TFLiteNode(Node):
    def __init__(self):
        super().__init__('tflite_node')
        self.get_logger().info("TFLite Node is starting...")

        # Load the TFLite model
        self.interpreter = tflite.Interpreter(model_path="./models/mnist.tflite")
        self.interpreter.allocate_tensors()

        # Get input and output details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # Timer to simulate inference
        self.timer = self.create_timer(1.0, self.run_inference)

    def run_inference(self):
        # Generate a random test input
        input_shape = self.input_details[0]['shape']
        input_data = np.random.rand(*input_shape).astype(np.float32)

        # Set the input tensor
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)

        # Perform inference
        self.interpreter.invoke()

        # Get the output tensor
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        self.get_logger().info(f"Model prediction: {output_data}")

def main(args=None):
    rclpy.init(args=args)
    node = TFLiteNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
