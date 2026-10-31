"""Read stamped world pose from Gazebo; no velocity publisher or TF broadcaster."""
import os

from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


class WorldPose(Node):
    def __init__(self):
        super().__init__('lab_world_pose')
        self.client = self.create_client(GetEntityState, '/lab/gazebo/get_entity_state')
        self.publisher = self.create_publisher(PoseStamped, '/lab/ground_truth', 10)
        self.pending = None
        self.ready = False
        self.create_timer(.1, self.query_world)

    def query_world(self):
        if not self.client.service_is_ready():
            self.get_logger().info('Waiting for Gazebo world-state service', throttle_duration_sec=5.)
            return
        if self.pending is not None and not self.pending.done():
            return
        request = GetEntityState.Request(name='burger', reference_frame='world')
        self.pending = self.client.call_async(request)
        self.pending.add_done_callback(self.received_world_pose)

    def received_world_pose(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warning('Gazebo has no burger world pose yet', throttle_duration_sec=5.)
                return
            message = PoseStamped(header=response.header, pose=response.state.pose)
            self.publisher.publish(message)
            if not self.ready:
                self.ready = True
                self.get_logger().info('WORLD REFERENCE READY: /lab/ground_truth (stamped simulation time)')
        except Exception as error:
            self.get_logger().error(f'World-state request failed: {error}', throttle_duration_sec=5.)


def main(args=None):
    if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Lab requires ROS_DOMAIN_ID=178 and ROS_LOCALHOST_ONLY=1')
    rclpy.init(args=args)
    node = None
    try:
        node = WorldPose()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
