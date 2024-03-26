import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
# insert in planning packages
import numpy as np

class MarkerPub(Node):
    def __init__(self):
        ### Create node
        super().__init__('turtle_marker_pub')
        self.publisher_ = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        ### Create Timer
        timer_period = 1. # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        # Set pose of marker

        marker_arr = MarkerArray()
        vals = [[],[],[],...] # insert here
            
        marker_arr.markers = []

        for i in range(len(vals)):
            marker_est = Marker()
            marker_est.header.frame_id = "xarm_gripper_base_link"
            marker_est.ns = "est_pose_"+str(i)
            marker_est.id = i
            marker_est.type = Marker.CUBE
            marker_est.action = Marker.ADD
            marker_est.pose.position.x = vals[i][0]
            marker_est.pose.position.y = vals[i][1]
            marker_est.pose.position.z = vals[i][2]
            marker_est.pose.orientation.x = vals[i][3]
            marker_est.pose.orientation.y = vals[i][4]
            marker_est.pose.orientation.z = vals[i][5]
            marker_est.pose.orientation.w = vals[i][6]
            marker_est.color.r, marker_est.color.g, marker_est.color.b = (0, 255, 0)
            marker_est.color.a = 0.5
            marker_est.scale.x, marker_est.scale.y, marker_est.scale.z = (0.05, 0.05, 0.05)
            marker_arr.markers.append(marker_est)

        
        self.publisher_.publish(marker_arr)
        
        self.get_logger().info('done')
        


def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPub()
    rclpy.spin(marker_publisher)
    # Destroy the node explicitly
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()