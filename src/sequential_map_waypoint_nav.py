import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from geometry_msgs.msg import PoseStamped
import csv
import json
import os
from rclpy.qos import qos_profile_services
from rclpy.parameter import Parameter

class SequentialMapWaypointNav(Node):
    def __init__(self):
        super().__init__('sequential_map_waypoint_nav')

        # パラメータの宣言と取得
        self.declare_parameter('map_waypoint_list_csv', 'map_waypoints.csv')
        self.declare_parameter('map_base_path', '$(find hokuyo_navigation2)/map/')
        self.declare_parameter('waypoint_base_path', '$(find hokuyo_navigation2)/waypoint/')
        self.declare_parameter('waypoint_tolerance_position', 0.2)
        self.declare_parameter('waypoint_tolerance_orientation', 0.2)

        self.map_waypoint_list_csv = self.get_parameter('map_waypoint_list_csv').get_parameter_value().string_value
        self.map_base_path = self.get_parameter('map_base_path').get_parameter_value().string_value
        self.waypoint_base_path = self.get_parameter('waypoint_base_path').get_parameter_value().string_value
        self.waypoint_tolerance_position = self.get_parameter('waypoint_tolerance_position').get_parameter_value().double_value
        self.waypoint_tolerance_orientation = self.get_parameter('waypoint_tolerance_orientation').get_parameter_value().double_value

        # Action Clientの作成
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.map_load_client = self.create_client(LoadMap, '/map_server/load_map', qos_profile_services)

        # 変数の初期化
        self.map_waypoint_sequences = []
        self.current_sequence_index = 0
        self.current_waypoint_index = 0
        self.is_navigating = False

        # データの読み込み
        self._load_map_waypoint_sequences()

        # ナビゲーションの開始
        self._start_navigation()

    def _load_map_waypoint_sequences(self):
        csv_path = os.path.join(os.getcwd(), self.map_waypoint_list_csv) # CSVファイルは実行ディレクトリにあると仮定
        try:
            with open(csv_path, 'r') as csvfile:
                reader = csv.reader(csvfile)
                next(reader, None)  # ヘッダー行をスキップ
                for row in reader:
                    if len(row) == 2:
                        map_file, waypoint_file = row
                        self.map_waypoint_sequences.append({'map': map_file, 'waypoint': waypoint_file})
                    else:
                        self.get_logger().warn(f"Invalid row in CSV: {row}")
        except FileNotFoundError:
            self.get_logger().error(f"CSV file not found: {csv_path}")
            rclpy.shutdown()

    def _load_waypoints_from_json(self, waypoint_file_name):
        json_path = os.path.join(self.waypoint_base_path, f"{waypoint_file_name}.json")
        try:
            with open(json_path, 'r') as f:
                waypoints_data = json.load(f)
                waypoints = []
                for waypoint in waypoints_data:
                    position = waypoint[0]
                    orientation_quat = waypoint[1]
                    pose = PoseStamped()
                    pose.header.frame_id = 'map' # 地図座標系を仮定
                    pose.pose.position.x = position[0]
                    pose.pose.position.y = position[1]
                    pose.pose.position.z = position[2]
                    pose.pose.orientation.x = orientation_quat[0]
                    pose.pose.orientation.y = orientation_quat[1]
                    pose.pose.orientation.z = orientation_quat[2]
                    pose.pose.orientation.w = orientation_quat[3]
                    waypoints.append(pose)
                return waypoints
        except FileNotFoundError:
            self.get_logger().error(f"Waypoint file not found: {json_path}")
            return None
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to decode JSON in: {json_path}")
            return None

    async def _load_new_map(self, map_file_name):
        self.get_logger().info(f"Loading new map: {map_file_name}")
        request = LoadMap.Request()
        request.map_url = os.path.join(self.map_base_path, f"{map_file_name}.yaml")

        if not self.map_load_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Map server not available")
            return False

        future = self.map_load_client.call_async(request)
        response = await future
        if response is not None and response.result:
            self.get_logger().info(f"Map {map_file_name} loaded successfully")
            return True
        else:
            self.get_logger().error(f"Failed to load map {map_file_name}")
            return False

    async def _send_goal(self, goal_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f"Navigating to: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)

        goal_handle = await send_goal_future
        if not goal_handle.accepted:
            self.get_logger().warn(f"Goal to ({goal_pose.pose.position.x}, {goal_pose.pose.position.y}) was rejected!")
            return False

        self.get_logger().info(f"Goal accepted to ({goal_pose.pose.position.x}, {goal_pose.pose.position.y})")
        get_result_future = goal_handle.get_result_async()
        result = await get_result_future
        self.get_logger().info(f"Navigation to ({goal_pose.pose.position.x}, {goal_pose.pose.position.y}) finished: {result.result.success}")
        return result.result.success

    async def _navigate_sequence(self):
        if self.current_sequence_index >= len(self.map_waypoint_sequences):
            self.get_logger().info("All map and waypoint sequences completed. Shutting down.")
            self.is_navigating = False
            rclpy.shutdown()
            return

        current_sequence = self.map_waypoint_sequences[self.current_sequence_index]
        map_file_name = current_sequence['map']
        waypoint_file_name = current_sequence['waypoint']

        # 新しい地図をロード
        if not await self._load_new_map(map_file_name):
            self.get_logger().error(f"Failed to load map {map_file_name}. Aborting sequence.")
            self.current_sequence_index += 1
            self.current_waypoint_index = 0
            await self._navigate_sequence() # 次のシーケンスへ
            return

        # ウェイポイントをロード
        waypoints = self._load_waypoints_from_json(waypoint_file_name)
        if waypoints is None or not waypoints:
            self.get_logger().warn(f"No valid waypoints found in {waypoint_file_name}. Skipping sequence.")
            self.current_sequence_index += 1
            self.current_waypoint_index = 0
            await self._navigate_sequence() # 次のシーケンスへ
            return

        self.get_logger().info(f"Starting navigation on map '{map_file_name}' with {len(waypoints)} waypoints.")

        for i in range(self.current_waypoint_index, len(waypoints)):
            self.current_waypoint_index = i
            goal_pose = waypoints[i]
            success = await self._send_goal(goal_pose)
            if not success:
                self.get_logger().warn(f"Navigation to waypoint {i+1} failed. Continuing to the next waypoint in the sequence.")

        # 現在のシーケンスが完了
        self.get_logger().info(f"Finished all waypoints for map '{map_file_name}'.")
        self.current_sequence_index += 1
        self.current_waypoint_index = 0
        await self._navigate_sequence() # 次のシーケンスへ

    def _start_navigation(self):
        if self.map_waypoint_sequences and not self.is_navigating:
            self.is_navigating = True
            self.future = self._navigate_sequence()

def main(args=None):
    rclpy.init(args=args)
    node = SequentialMapWaypointNav()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()