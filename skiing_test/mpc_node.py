import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String  # Adjust the message type based on your needs
from trajectory_gen.trajectory_gen.new_3004 import run_mpc  # Import the run_mpc function

class MPCPublisherNode(Node):
    def __init__(self):
        super().__init__('mpc_publisher_node')
        self.publisher_ = self.create_publisher(String, 'mpc_result', 10)
        self.timer = self.create_timer(1.0, self.publish_mpc_result)  # Publish every second
        # load data from the file joint_trajectory.npy
        self.joint_trajectory = np.load('joint_trajectory.npy', allow_pickle=True)
        print(self.joint_trajectory.shape)
        self.count = 0
    def publish_mpc_result(self):

            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error while running run_mpc: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MPCPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()