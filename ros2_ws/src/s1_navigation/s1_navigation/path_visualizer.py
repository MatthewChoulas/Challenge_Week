"""Display the planned path as an RViz Path and a Gazebo marker model."""

import math

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity


MODEL_NAME = 'planned_path_marker'
MAX_SEGMENTS = 500
GAZEBO_PATH_Z_OFFSET = -0.22
GAZEBO_PATH_THICKNESS = 0.08


def make_path_sdf(path):
    """Create one collision-free model containing blue boxes along a 3D path."""
    if len(path.poses) < 2:
        return None
    stride = max(1, math.ceil((len(path.poses) - 1) / MAX_SEGMENTS))
    indices = list(range(0, len(path.poses) - 1, stride))
    if indices[-1] != len(path.poses) - 1:
        indices.append(len(path.poses) - 1)

    visuals = []
    for visual_id, (first, second) in enumerate(zip(indices, indices[1:])):
        a = path.poses[first].pose.position
        b = path.poses[second].pose.position
        dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
        horizontal = math.hypot(dx, dy)
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length < 1e-6:
            continue
        yaw = math.atan2(dy, dx)
        pitch = -math.atan2(dz, horizontal)
        visuals.append(f'''<visual name="segment_{visual_id}">
          <pose>{(a.x+b.x)/2} {(a.y+b.y)/2} {(a.z+b.z)/2 + GAZEBO_PATH_Z_OFFSET} 0 {pitch} {yaw}</pose>
          <geometry><box><size>{length} 1.5 {GAZEBO_PATH_THICKNESS}</size></box></geometry>
          <material><ambient>0.05 0.25 1 1</ambient>
          <diffuse>0.05 0.25 1 1</diffuse></material>
        </visual>''')
    if not visuals:
        return None
    return (f'<sdf version="1.9"><model name="{MODEL_NAME}"><static>true</static>'
            f'<link name="path">{"".join(visuals)}</link></model></sdf>')


class PathVisualizer(Node):

    def __init__(self):
        super().__init__('path_visualizer')
        self.subscription = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10)
        self.create_client_ = self.create_client(
            SpawnEntity, '/world/usgs_utah/create')
        self.remove_client = self.create_client(
            DeleteEntity, '/world/usgs_utah/remove')
        self.pending_sdf = None
        self.request_pending = False
        self.model_exists = False
        self.update_requested = False
        self.timer = self.create_timer(0.25, self.update_gazebo)

    def path_callback(self, message):
        self.pending_sdf = make_path_sdf(message)
        self.update_requested = True

    def update_gazebo(self):
        if self.request_pending or not self.update_requested:
            return
        if self.model_exists:
            if not self.remove_client.service_is_ready():
                return
            request = DeleteEntity.Request()
            request.entity.name = MODEL_NAME
            request.entity.type = request.entity.MODEL
            self.request_pending = True
            future = self.remove_client.call_async(request)
            future.add_done_callback(self.remove_done)
            return
        if self.pending_sdf is None:
            self.update_requested = False
            return
        if not self.create_client_.service_is_ready():
            return
        request = SpawnEntity.Request()
        request.entity_factory.name = MODEL_NAME
        request.entity_factory.sdf = self.pending_sdf
        request.entity_factory.pose.orientation.w = 1.0
        self.request_pending = True
        future = self.create_client_.call_async(request)
        future.add_done_callback(self.create_done)

    def remove_done(self, future):
        self.request_pending = False
        try:
            self.model_exists = not future.result().success
        except Exception as error:
            self.get_logger().warning(f'Could not remove old Gazebo path: {error}')

    def create_done(self, future):
        self.request_pending = False
        try:
            if future.result().success:
                self.model_exists = True
                self.update_requested = False
                self.get_logger().info('Displayed planned path in Gazebo')
            else:
                self.get_logger().warning('Gazebo rejected planned path marker')
        except Exception as error:
            self.get_logger().warning(f'Could not create Gazebo path: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
