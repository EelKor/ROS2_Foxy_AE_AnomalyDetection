import argparse
import time
import os
import numpy as np
import tensorflow as tf
import rclpy

from PIL import Image
from tensorflow.python.client import device_lib
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, Float64
from ament_index_python.packages import get_package_share_directory

class AutoencoderROS(Node):
    def __init__(self):
        super().__init__('AnomalyDetection_AE')
        self.init_pubsub()
        self.init_value()
        self.init_param()

        self.timer = self.create_timer(0.01, self.run)

    def init_pubsub(self):        
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/dp/out/autoencoder_inputs',
            self.listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(Float64, '/ad/out/reconstruction_error', 10)
                
    def init_value(self):
        maxAx_ = 0.0263608272674425
        maxAy_ = 0.0264559128084445
        maxAz_ = 23.5368741740405

        maxP_ = 0.557505493926602
        maxQ_ = 0.552249716931726
        maxR_ = 0.01424845041569

        maxPhi_   = 0.265496976180982
        maxTheta_ = 0.27155233388245
        maxPsi_   = 0.785188210141704

        self.min_Ax = -0.0268811654409069
        self.min_Ay = -0.0265703248054632
        self.min_Az = 5.33848345283791

        self.min_P = -0.559515425220955
        self.min_Q = -0.55590503976407
        self.min_R = -0.37013956720171

        self.min_Phi   = -0.266343899153498
        self.min_Theta = -0.27524570557516
        self.min_Psi   = -0.0220543220534516

        self.err_Ax = maxAx_ - self.min_Ax
        self.err_Ay = maxAy_ - self.min_Ay
        self.err_Az = maxAz_ - self.min_Az

        self.err_P = maxP_ - self.min_P
        self.err_Q = maxQ_ - self.min_Q
        self.err_R = maxR_ - self.min_R

        self.err_Phi   = maxPhi_ - self.min_Phi
        self.err_Theta = maxTheta_ - self.min_Theta
        self.err_Psi   = maxPsi_ - self.min_Psi

        self.threshold = 0.12027066344921587
        self.stdD      = 0.008686784416758167
        self.maxMSE    = 0.1115838790324577
        self.margin    = -1.0

        self.inputData = np.array([])

    def init_param(self):
        self.model_path = '/home/wsl/ws_anomaly_detection/src/ros_anomaly/model/model.tflite'

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
            # Load TFLite model
            interpreter = tf.lite.Interpreter(model_path=self.model_path)
            interpreter.allocate_tensors()

            # Get input/output tensors
            input_details = interpreter.get_input_details()
            output_details = interpreter.get_output_details()

            inputData_ = tf.expand_dims(self.inputData, axis=0)
            
            # Set input tensor
            interpreter.set_tensor(input_details[0]['index'], inputData_)

            # Run inference
            interpreter.invoke()

            # Get output tensor
            output_data = interpreter.get_tensor(output_details[0]['index'])
            output_data = np.squeeze(output_data,axis=0)
            mse = np.mean(np.power((inputData_ - output_data), 2), axis=1)
            result_msg = Float64()
            result_msg.data = float(mse)

            self.publisher_.publish(result_msg)

def main(args=None):
    rclpy.init(args=args)

    _run = AutoencoderROS()

    while rclpy.ok():
        rclpy.spin(_run)

    _run.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
