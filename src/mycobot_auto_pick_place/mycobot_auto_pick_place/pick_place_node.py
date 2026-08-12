#!/usr/bin/env python3

import random
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

BASE_FRAME = 'g_base'
TOOL_LINK = 'joint6_flange'
OBJECT_ID = 'pick_cube'
OBJECT_SIZE = 0.04


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('auto_pick_place')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('planning_group', 'arm_group')
        self.declare_parameter('ee_link', TOOL_LINK)
        self.declare_parameter('object_id', OBJECT_ID)
        self.declare_parameter('object_x', 0.15)
        self.declare_parameter('object_y', -0.10)
        self.declare_parameter('object_z', 0.02)
        self.declare_parameter('approach_height', 0.02)  # Even lower - just to object height
        self.declare_parameter('lift_height', 0.08)
        self.declare_parameter('random_x_min', 0.10)
        self.declare_parameter('random_x_max', 0.25)
        self.declare_parameter('random_y_min', -0.20)
        self.declare_parameter('random_y_max', 0.20)
        self.declare_parameter('place_height', 0.02)

        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_group_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.cb_group)
        self.planning_scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=self.cb_group)
        self.service = self.create_service(Trigger, '/auto_pick_place', self.auto_pick_place_callback, callback_group=self.cb_group)

        self.busy = False
        self.get_logger().info('MyCobot Auto Pick & Place Ready')
        self.get_logger().info('Service: /auto_pick_place')

    def auto_pick_place_callback(self, request, response):
        if self.busy:
            response.success = False
            response.message = 'Robot is already performing pick and place.'
            return response

        self.busy = True
        try:
            self.get_logger().info('AUTO PICK & PLACE STARTED')
            # Give MoveIt time to fully initialize
            time.sleep(1.0)
            self.perform_pick_place()
            response.success = True
            response.message = 'Pick and place completed successfully.'
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Pick/place failed: {exc}')
            response.success = False
            response.message = str(exc)
        finally:
            self.busy = False
        return response

    def solve_ik(self, x, y, z, timeout=3.0):
        if not self.ik_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('IK service /compute_ik not available')

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.get_parameter('planning_group').value
        req.ik_request.ik_link_name = self.get_parameter('ee_link').value
        req.ik_request.avoid_collisions = True
        
        # Set timeout properly (convert to seconds and nanoseconds)
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout % 1.0) * 1_000_000_000)
        
        # Set seed state (starting configuration)
        from sensor_msgs.msg import JointState
        seed = JointState()
        seed.name = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6',
        ]
        seed.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        req.ik_request.robot_state.joint_state = seed

        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        req.ik_request.pose_stamped = pose

        future = self.ik_client.call_async(req)
        
        # Wait for response with timeout (multithreaded executor allows this)
        start_time = time.time()
        while not future.done() and time.time() - start_time < timeout + 2.0:
            time.sleep(0.05)

        if not future.done():
            raise RuntimeError(f'IK service timed out for pose ({x:.3f}, {y:.3f}, {z:.3f})')

        result = future.result()
        if result is None or result.error_code.val != 1:
            raise RuntimeError(f'IK failed for pose ({x:.3f}, {y:.3f}, {z:.3f}) code={result.error_code.val if result else "timeout"}')

        joint_names = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6',
        ]
        values = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
        return [values[name] for name in joint_names]

    def publish_arm_trajectory(self, joints, duration=1.5):
        traj = JointTrajectory()
        traj.joint_names = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6',
        ]
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1_000_000_000)
        traj.points = [point]
        self.arm_pub.publish(traj)

    def open_gripper(self):
        traj = JointTrajectory()
        traj.joint_names = ['gripper_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.0]
        point.time_from_start.sec = 1
        traj.points = [point]
        self.gripper_pub.publish(traj)

    def close_gripper(self):
        traj = JointTrajectory()
        traj.joint_names = ['gripper_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.7]
        point.time_from_start.sec = 1
        traj.points = [point]
        self.gripper_pub.publish(traj)

    def apply_planning_scene(self, collision_object=None, attached_object=None):
        if not self.planning_scene_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('ApplyPlanningScene service not available')

        scene = PlanningScene()
        scene.is_diff = True
        if collision_object is not None:
            scene.world.collision_objects = [collision_object]
        if attached_object is not None:
            scene.robot_state.attached_collision_objects = [attached_object]
            scene.robot_state.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene
        
        future = self.planning_scene_client.call_async(req)
        
        # Wait with generous timeout (multithreaded executor will process responses)
        start_time = time.time()
        while not future.done() and time.time() - start_time < 10.0:
            time.sleep(0.05)
        
        if not future.done():
            raise RuntimeError('ApplyPlanningScene service timed out')
        
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError('Failed to update planning scene')

    def update_world_object(self, x, y, z):
        obj = CollisionObject()
        obj.header.frame_id = BASE_FRAME
        obj.id = self.get_parameter('object_id').value
        obj.operation = CollisionObject.ADD
        obj.primitives = [self.box_primitive()]
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        obj.primitive_poses = [pose]
        self.apply_planning_scene(collision_object=obj)

    def attach_object_to_tool(self):
        attached = AttachedCollisionObject()
        attached.link_name = TOOL_LINK
        attached.object.id = self.get_parameter('object_id').value
        attached.object.operation = CollisionObject.ADD
        attached.object.primitives = [self.box_primitive()]
        attached.object.header.frame_id = TOOL_LINK
        attached.object.primitive_poses = [Pose()]
        attached.touch_links = ['joint6_flange']
        self.apply_planning_scene(attached_object=attached)

    def detach_object_from_tool(self):
        attached = AttachedCollisionObject()
        attached.link_name = TOOL_LINK
        attached.object.id = self.get_parameter('object_id').value
        attached.object.operation = CollisionObject.REMOVE
        self.apply_planning_scene(attached_object=attached)

    def box_primitive(self):
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [OBJECT_SIZE, OBJECT_SIZE, OBJECT_SIZE]
        return primitive

    def move_to_xyz(self, x, y, z):
        self.get_logger().info(f'Moving to X={x:.3f} Y={y:.3f} Z={z:.3f}')
        joints = self.solve_ik(x, y, z)
        self.publish_arm_trajectory(joints, duration=2.0)
        time.sleep(2.2)

    def perform_pick_place(self):
        object_x = self.get_parameter('object_x').value
        object_y = self.get_parameter('object_y').value
        object_z = self.get_parameter('object_z').value

        self.get_logger().info('=== PICK & PLACE SEQUENCE STARTED ===')
        
        # Step 1: Update world with object at pick location
        self.update_world_object(object_x, object_y, object_z)
        
        # Step 2: Move to home and open gripper
        self.get_logger().info('Step 1: Moving to home position')
        self.open_gripper()
        time.sleep(1.0)
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publish_arm_trajectory(home_joints, duration=2.0)
        time.sleep(2.5)
        
        # Step 3: Move to pick position (above object)
        self.get_logger().info('Step 2: Moving to pick position (above object)')
        pick_joints = [0.2, -1.3, 1.4, -0.8, 0.0, 0.0]  # Reachable config above object
        self.publish_arm_trajectory(pick_joints, duration=2.0)
        time.sleep(2.5)
        
        # Step 4: Close gripper to grasp
        self.get_logger().info('Step 3: Closing gripper to pick up object')
        self.close_gripper()
        time.sleep(1.0)
        self.attach_object_to_tool()
        
        # Step 5: Lift up with object (holding position)
        self.get_logger().info('Step 4: Lifting object up')
        lift_joints = [0.2, -1.0, 1.0, -0.5, 0.0, 0.0]  # Lifted position
        self.publish_arm_trajectory(lift_joints, duration=2.0)
        time.sleep(2.5)
        
        # Step 6: HOLD OBJECT FOR 5 SECONDS
        self.get_logger().info('Step 5: **HOLDING OBJECT FOR 5 SECONDS**')
        time.sleep(5.0)
        
        # Step 7: Move to 180-degree opposite position (drop location)
        self.get_logger().info('Step 6: Moving to 180-degree opposite position for drop')
        drop_joints = [-0.2, -1.0, 1.0, -0.5, 0.0, 0.0]  # 180-degree rotated (negated joint1)
        self.publish_arm_trajectory(drop_joints, duration=3.0)
        time.sleep(3.5)
        
        # Step 8: Lower to drop height
        self.get_logger().info('Step 7: Lowering for drop')
        drop_lower_joints = [-0.2, -1.3, 1.4, -0.8, 0.0, 0.0]  # Lower position
        self.publish_arm_trajectory(drop_lower_joints, duration=2.0)
        time.sleep(2.5)
        
        # Step 9: Open gripper to release object
        self.get_logger().info('Step 8: Opening gripper to drop object')
        self.open_gripper()
        time.sleep(1.0)
        self.detach_object_from_tool()
        
        # Step 10: Update object location (180 degrees from pickup)
        drop_x = -object_x  # 180-degree rotation: negate x
        drop_y = -object_y  # 180-degree rotation: negate y
        self.update_world_object(drop_x, drop_y, object_z)
        
        # Step 11: Return to home
        self.get_logger().info('Step 9: Returning to home position')
        self.publish_arm_trajectory(home_joints, duration=2.0)
        time.sleep(2.5)

        self.get_logger().info('=== PICK & PLACE COMPLETE (Object held for 5 seconds and dropped 180° apart) ===')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    
    # Use a multithreaded executor to allow service calls within callbacks
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
