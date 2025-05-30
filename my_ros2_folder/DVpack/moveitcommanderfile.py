
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ros2_igtl_bridge.msg import Transform
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, WorkspaceParameters
from shape_msgs.msg import SolidPrimitive

#create subscriber listening to channel /IGTL_TRANSFORM_IN
class TransformToMoveItBridge(Node):
    def __init__(self):
        super().__init__('transform_to_moveit_bridge')
        self.sub = self.create_subscription(
            Transform,
            '/IGTL_TRANSFORM_IN',
            self.transform_callback,
            1
        )
        #action client for MoveGroup. note that there is a difference between move_group and MoveGroup. MoveGroup is a 
        #type of message and move_group is the trajectory generator. Messages are sent through /move_action to move_group in the form of 
        #MoveGroup
        self._mg_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Waiting for move_action server...')
        self._mg_client.wait_for_server()
        self.get_logger().info('Connected to MoveGroup action server')

    def transform_callback(self, msg: Transform):
        #creating a pose type PoseStamped() from the transform
        pose = PoseStamped()
        if hasattr(msg, 'header') and msg.header.frame_id:
            pose.header = msg.header
        else:
            #setting planning frame in world 
            pose.header.frame_id = 'world'
            pose.header.stamp = self.get_clock().now().to_msg()

        #extracting position and orientation term from the tranformation matrix
        pose.pose.position.x = msg.transform.translation.x
        pose.pose.position.y = msg.transform.translation.y
        pose.pose.position.z = msg.transform.translation.z
        pose.pose.orientation = msg.transform.rotation

        
        goal = MoveGroup.Goal()
        goal.request.group_name = 'my_group'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 120.0

        # icreate workspace 
        wp = WorkspaceParameters()
        wp.header = pose.header
        wp.min_corner.x, wp.min_corner.y, wp.min_corner.z = -10.0, -10.0, 0.0
        wp.max_corner.x, wp.max_corner.y, wp.max_corner.z =  10.0, 10.0, 10.0
        goal.request.workspace_parameters = wp

        # position constraint, and acceptable error (1 cm)
        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = 'end_effector'
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]
        pc.constraint_region.primitives.append(box)
        pc.constraint_region.primitive_poses.append(pose.pose)
        pc.weight = 1.0

        #acceptable orientation error 
        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = 'end_effector'
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
      
        cons = Constraints()
        cons.position_constraints = [pc]
        cons.orientation_constraints = [oc]
        goal.request.goal_constraints = [cons]

        # send the MoveGroup.Goal()
        self.get_logger().info(
            f"[DBG] Sending MoveGroup goal → "
            f"x={pose.pose.position.x:.3f}, "
            f"y={pose.pose.position.y:.3f}, "
            f"z={pose.pose.position.z:.3f}"
        )
        send_future = self._mg_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_callback)


    #printing terminal messages 
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by MoveGroup')
            return
        self.get_logger().info('Goal accepted, waiting for result…')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('MoveGroup execution succeeded')
        else:
            self.get_logger().warn(f'MoveGroup failed (error code {result.error_code.val})')

def main():
    rclpy.init()
    bridge = TransformToMoveItBridge()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
