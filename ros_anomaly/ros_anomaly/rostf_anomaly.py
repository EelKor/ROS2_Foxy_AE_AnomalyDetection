import argparse
import time
import os
import numpy as np
import tensorflow as tf
import rclpy

from PIL import Image
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, Float64, Int32
from ament_index_python.packages import get_package_share_directory

class AutoencoderROS(Node):
    def __init__(self):
        super().__init__('AnomalyDetection_AE')
        self.init_pubsub()
        self.init_value()
        self.init_param()
        self.init_model()
        self.timer = self.create_timer(0.01, self.run)

    def init_pubsub(self):        
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/dp/out/autoencoder_inputs',
            self.listener_callback,
            10)
        
        self.input_output_publisher_ = self.create_publisher(Float64MultiArray, '/autoencoder/input_outputs', 10)
        self.size_publisher_ = self.create_publisher(Int32, '/autoencoder/size', 10)

    def init_value(self):
        maxAx_ = 1.69
        maxAy_ = 1.42
        maxAz_ = -0.0102182215826467

        maxP_ = 0.99
        maxQ_ = 1.34
        maxR_ = 0.91

        maxPhi_   = 0.21
        maxTheta_ = 0.22
        maxPsi_   = 3.14

        self.min_Ax = -1.32
        self.min_Ay = -1.06
        self.min_Az = -32.366

        self.min_P = -1.07
        self.min_Q = -1.20
        self.min_R = -0.38

        self.min_Phi   = -0.22
        self.min_Theta = -0.20
        self.min_Psi   = -3.14

        self.err_Ax = maxAx_ - self.min_Ax
        self.err_Ay = maxAy_ - self.min_Ay
        self.err_Az = maxAz_ - self.min_Az

        self.err_P = maxP_ - self.min_P
        self.err_Q = maxQ_ - self.min_Q
        self.err_R = maxR_ - self.min_R

        self.err_Phi   = maxPhi_ - self.min_Phi
        self.err_Theta = maxTheta_ - self.min_Theta
        self.err_Psi   = maxPsi_ - self.min_Psi

        self.threshold = 0.2968564678921582
        self.stdD      = 0.007407137168589778
        self.maxMSE    = 0.1167213326321647
        self.margin    = 5.0

        self.inputData = np.array([])
        self.outputData = np.array([])

    def init_param(self):
        self.model_path = '/root/ros2_ws/src/ros_anomaly/model/model.tflite'
<<<<<<< HEAD
=======

    def init_model(self):
        self.interpreter = tf.lite.Interpreter(model_path=self.model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
>>>>>>> ksas

    def NormalizationMinMax(self, data):        
        input_Ax = (data[1] - self.min_Ax) / (self.err_Ax)
        input_Ay = (data[2] - self.min_Ay) / (self.err_Ay)
        input_Az = (data[3] - self.min_Az) / (self.err_Az)

        input_P = (data[4] - self.min_P) / (self.err_P)
        input_Q = (data[5] - self.min_Q) / (self.err_Q)
        input_R = (data[6] - self.min_R) / (self.err_R)

        input_Phi   = (data[7] - self.min_Phi) / (self.err_Phi)
        input_Theta = (data[8] - self.min_Theta) / (self.err_Theta)
        input_Psi   = (data[9] - self.min_Psi) / (self.err_Psi)

        self.inputData = np.array([input_Ax, input_Ay, input_Az, input_P, input_Q, input_R, input_Phi, input_Theta, input_Psi], np.float32)
        
    def listener_callback(self, msg):
        self.NormalizationMinMax(msg.data)

    def run(self):
        if self.inputData.size != 0: # Very Very Important condition, tip from D.H.Seo

            input_data_batch = np.expand_dims(self.inputData, axis=0).astype(np.float32)

            self.interpreter.set_tensor(self.input_details[0]['index'], input_data_batch)
            self.interpreter.invoke()
            output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
            output_data = np.squeeze(output_data, axis=0)
    
            ae_input_output_msg = Float64MultiArray()
            ae_input_output_msg.data = self.inputData.tolist() +[-9999.0] + output_data.tolist()
            self.input_output_publisher_.publish(ae_input_output_msg)

            # Publish the size of the input data
            size_msg = Int32()
            size_msg.data = len(self.inputData)
            self.size_publisher_.publish(size_msg)
            

def main(args=None):
    rclpy.init(args=args)
    _run = AutoencoderROS()

    while rclpy.ok():
        rclpy.spin(_run)

    _run.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
