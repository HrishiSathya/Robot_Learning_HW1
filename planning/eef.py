import math
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FrameListener(Node):

    def __init__(self):
        super().__init__('eef_link_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'link_eef').get_parameter_value().string_value
        
        self.to_frame = self.declare_parameter(
          'to_frame', 'world').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(String, 'eef_traj', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(0.7, self.on_timer)

        self.store = []

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = self.to_frame

        # if self.turtle_spawning_service_ready:
        #     if self.turtle_spawned:
        #         # Look up for the transformation between target_frame and turtle2 frames
        #         # and send velocity commands for turtle2 to reach target_frame
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        trans = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])

        # self.store.append(trans)
        with open('eefs/eef_env1_g1_t3.npy', 'ab') as f:
            np.save(f, trans)

        self.get_logger().info('The translation vector is %s' % trans)

        msg = String()
        msg.data = str(trans)
        
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
