#!/usr/bin/env python3
"""
Spawns a square pickable object into the MoveIt planning scene.
Run this AFTER demo.launch.py is up and move_group is ready.
"""
import sys
import time
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

OBJECT_ID = "pick_cube"
OBJECT_SIZE = 0.04  # 40mm cube
FRAME_ID = "g_base"

# Default spawn location: in front of the robot, reachable
DEFAULT_X = 0.15
DEFAULT_Y = -0.10
DEFAULT_Z = 0.02


class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /apply_planning_scene service...')

    def spawn(self, x=DEFAULT_X, y=DEFAULT_Y, z=DEFAULT_Z):
        obj = CollisionObject()
        obj.header.frame_id = FRAME_ID
        obj.id = OBJECT_ID
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [OBJECT_SIZE, OBJECT_SIZE, OBJECT_SIZE]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        obj.primitives = [primitive]
        obj.primitive_poses = [pose]

        scene = PlanningScene()
        scene.world.collision_objects = [obj]
        scene.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(
                f'Object "{OBJECT_ID}" spawned at ({x:.3f}, {y:.3f}, {z:.3f}) in frame "{FRAME_ID}"')
        else:
            self.get_logger().error('Failed to spawn object!')


def main():
    rclpy.init(args=None)
    node = ObjectSpawner()

    x = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_X
    y = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_Y
    z = float(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_Z

    node.spawn(x, y, z)
    time.sleep(0.5)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
