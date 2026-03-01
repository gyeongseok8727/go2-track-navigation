#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

import subprocess
from collections import deque

import time

def make_pose(x, y, ori_z, ori_w):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    # 노드 clock을 써도 되고, 여기선 간단히 현재 시간으로
    pose.header.stamp = rclpy.clock.Clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = ori_z
    pose.pose.orientation.w = ori_w
    return pose

class FollowWaypointsClient(Node):
    def __init__(self):
        super().__init__('follow_waypoints_client')
        self._wp_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self._goal_waypoints_len = 0

        # (poses_list, mission_cmd_list_or_None) 를 담는 큐
        # poses_list: [PoseStamped, ...]  (한 번의 FollowWaypoints 목표에 들어갈 포즈들)
        # mission_cmd: 성공 직후 실행할 외부 커맨드 (list) 또는 None
        self.queue = deque()
        self._active = False  # 현재 goal 진행 중 여부

    def enqueue_goal(self, poses_list, mission_cmd=None):
        """다음에 보낼 FollowWaypoints 목표를 큐에 추가."""
        self.queue.append((poses_list, mission_cmd))
        # 아직 아무 것도 진행 중이 아니면 즉시 시작
        if not self._active:
            self._dequeue_and_send_next()

    def _dequeue_and_send_next(self):
        if not self.queue:
            self._active = False
            self.get_logger().info('All waypoint goals completed.')
            rclpy.shutdown()
            return

        poses, mission_cmd = self.queue.popleft()
        self._send_goal(poses, mission_cmd)

    def _send_goal(self, poses, mission_cmd):
        self._active = True
        self._goal_waypoints_len = len(poses)
        self._pending_mission_cmd = mission_cmd

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.get_logger().info('Waiting for /follow_waypoints action server...')
        self._wp_client.wait_for_server()

        self.get_logger().info(f'Sending {len(poses)} waypoints...')
        future = self._wp_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('FollowWaypoints goal rejected')
            # 거부되면 다음 것으로 넘어가거나 종료
            self._active = False
            self._dequeue_and_send_next()
            return

        self.get_logger().info('FollowWaypoints goal accepted')
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        time.sleep(0.5)
        # current_waypoint는 0-based index
        # self.get_logger().info(f'Currently at waypoint {fb.current_waypoint + 1}/{self._goal_waypoints_len}')

    def _result_cb(self, future):
        result_wrapper = future.result()
        status = result_wrapper.status
        result  = result_wrapper.result  # nav2_msgs/action/FollowWaypoints_Result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('FollowWaypoints SUCCEEDED')

            # 성공 후 외부 커맨드가 지정되어 있으면 실행
            if self._pending_mission_cmd:
                try:
                    subprocess.run(self._pending_mission_cmd, check=True)
                    self.get_logger().info('External command finished successfully.')
                except subprocess.CalledProcessError as e:
                    self.get_logger().error(f'External command failed: {e}')

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('FollowWaypoints CANCELED')
        else:
            self.get_logger().error(f'FollowWaypoints FAILED (status={status})')
            # FollowWaypoints_Result 필드는 missed_waypoints 가 맞음 (humble 기준)
            if hasattr(result, 'missed_waypoints') and result.missed_waypoints:
                self.get_logger().error(f'Missed waypoints: {list(result.missed_waypoints)}')

        # 현 goal 종료 → 다음 goal로
        self._active = False
        self._dequeue_and_send_next()

def main(args=None):
    rclpy.init(args=args)
    node = FollowWaypointsClient()

    subprocess.run(["ros2", "topic", "pub", "--once", "mission", "std_msgs/msg/Int32", "data: 7"], check=True)

    wp5 = [
        make_pose(-7.727145195007324, 1.4872357845306396, -0.30633198526130717, 0.9519247421964965),
        make_pose(-6.88930082321167, 1.0355812311172485, 0.4600616826559009, 0.8878869568542052),
        make_pose(-7.282036781311035, 2.2988979816436768, 0.8064215737026346, 0.5913410568081388),
        make_pose(-5.6848907470703125, 3.0450987815856934, 0.4758651137767629, 0.8795182735396851),
        make_pose(-4.286280066807501, 4.19284674633661, -0.35036445997223237, 0.9366134449111683),
    ] # make_pose(-5.645610332489014, 4.858250617980957, 0.09645386945647307, 0.9953374558745761),
    #make_pose(-6.529285907745361, 1.7294212579727173, 0.9480340803773177, 0.3181687955207635),
    #make_pose(-6.391822814941406, 2.868367910385132, 0.10496743557551136, 0.9944756595657336),
    #make_pose(-5.81580114364624, 3.902582883834839, 0.73873153802662, 0.6739997883714983),
    mission_cmd5 = ["ros2", "topic", "pub", "--once", "mission", "std_msgs/msg/Int32", "data: 8"]
    node.enqueue_goal(wp5, mission_cmd5)


    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 필요하면 여기서만 종료
        rclpy.shutdown()

if __name__ == '__main__':#
    main()
