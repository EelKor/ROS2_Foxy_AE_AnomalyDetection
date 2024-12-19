import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from collections import deque

class Float64Subscriber(Node):
    def __init__(self):
        super().__init__('float64_graph_node')
        
        # Create a subscriber
        self.subscription = self.create_subscription(
            Float64,
            '/ad/out/reconstruction_error',  # Replace with your topic name
            self.listener_callback,
            10)
        
        # Deque to store recent data points
        self.data = deque(maxlen=100)  # Store up to 100 points
        
        # Matplotlib figure setup
        plt.ion()  # Interactive mode
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-')  # Initialize plot
        
        self.ax.set_title('Reconstruction Error')
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('MSE')

        # Fix y-axis scale
        self.ax.set_ylim(-0.5, 100)  

    def listener_callback(self, msg):
        # Append new data to the deque
        self.data.append(msg.data)
        
        # Update graph
        self.update_plot()
    
    def update_plot(self):
        # Update line data
        self.line.set_xdata(range(len(self.data)))
        self.line.set_ydata(self.data)
        
        # Adjust plot limits
        self.ax.relim()
        self.ax.autoscale_view()
        
        # Redraw the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = Float64Subscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()  # Turn off interactive mode
        plt.show()  # Keep the final plot open

if __name__ == '__main__':
    main()
